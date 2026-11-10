
"""Camera-based PID controller for a 3-servo Stewart platform.

Uses:
    ball_detection.py
    kinematics.py
    stewart_config.json
    kinematics_config.json

Arduino: PCA9685 channels 0, 3, 7.
Serial: comma-separated angles in tenths of a degree, baud rate from
stewart_config.json (115200). Requires the servo_controller.ino in this
folder to be uploaded (the older sketch only accepts whole degrees at 9600).

Usage:
    python pid_controller.py                 # monitor only, no servo commands
    python pid_controller.py --live --log    # balance with the default gains
    python pid_controller.py --live --kp 20 --ki 8 --kd 25 --max-tilt 5 --log

Units: kp is degrees of tilt per metre of ball offset, kd is degrees per
(metre/second), ki is degrees per (metre*second).

Tuning notes (from the logged runs of 2026-11-10, see CHANGES.md):
    * The loop has roughly 0.2 s of dead time (camera + serial + servos) and
      the ball accelerates about 0.07 m/s^2 per commanded degree. That limits
      how much gain is usable: kp much above ~40 makes the ball orbit the
      centre and grow outward, whatever kd is.
    * kd needs to be about as large as kp (kd/kp of 0.8-1.2 s).
    * Run python analyze_log.py after a --log run to re-measure these.

Controls:
    SPACE = Start/pause balancing
    R     = Return to neutral / recover from Arduino error
    Q/ESC = Quit

The controller continues correcting near the platform edge.
Arduino errors pause balancing but do not require restarting.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time

from collections import deque
from datetime import datetime
from pathlib import Path

import cv2

from ball_detection import BallDetector
from kinematics import KinematicsError, StewartKinematics


ROOT = Path(__file__).resolve().parent

# Servo angles are sent in tenths of a degree. Whole degrees are roughly
# 0.7 deg of plate tilt per step on this mechanism, which is too coarse.
SERIAL_DECIMALS = 1


# ==================================================
# PID CONTROLLER
# ==================================================

class PID:
    """One axis of the controller. Output is in degrees of tilt."""

    def __init__(
        self,
        kp,
        ki,
        kd,
        derivative_tau=0.08,
        integral_limit_deg=2.5
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Low-pass time constant on the derivative. This filter is inside
        # the loop, so its lag adds directly to the loop delay: keep it as
        # small as the camera noise allows.
        self.derivative_tau = derivative_tau

        # Largest tilt the integral term alone may ask for.
        self.integral_limit_deg = integral_limit_deg

        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    @property
    def integral_term(self):
        return self.ki * self.integral

    def update(self, error, dt, integrate=True):

        dt = max(0.001, min(dt, 0.2))

        if self.previous_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (
                error - self.previous_error
            ) / dt

        self.previous_error = error

        # Filter derivative noise
        alpha = dt / (self.derivative_tau + dt)

        self.filtered_derivative += alpha * (
            raw_derivative - self.filtered_derivative
        )

        # Integral with anti-windup: the caller stops integration while
        # the tilt command is saturated, and the term itself is clamped.
        if integrate and self.ki > 0:

            self.integral += error * dt

            limit = self.integral_limit_deg / self.ki

            self.integral = max(
                -limit,
                min(limit, self.integral)
            )

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )


# ==================================================
# PLATFORM TILT LIMITS
# ==================================================

def limit_vector(roll, pitch, max_tilt):

    magnitude = math.hypot(roll, pitch)

    if magnitude > max_tilt:

        scale = max_tilt / magnitude

        return (
            roll * scale,
            pitch * scale
        )

    return roll, pitch


def reachable_tilt(solver, roll, pitch):
    """
    Find the largest reachable tilt in the requested direction.

    Reduces the commanded tilt when the calculated servo
    angles exceed calibrated limits.
    """

    try:

        angles = solver.calculate_servo_angles(
            roll, pitch
        )

        command = solver.format_serial_command(
            angles, SERIAL_DECIMALS
        )

        return roll, pitch, angles, command, False

    except KinematicsError:

        home_angles = solver.calculate_servo_angles(
            0.0, 0.0
        )

        home_command = solver.format_serial_command(
            home_angles, SERIAL_DECIMALS
        )

        best = (
            0.0,
            0.0,
            home_angles,
            home_command,
            True
        )

        low = 0.0
        high = 1.0

        for _ in range(22):

            scale = (low + high) * 0.5

            test_roll = roll * scale
            test_pitch = pitch * scale

            try:

                angles = solver.calculate_servo_angles(
                    test_roll, test_pitch
                )

                command = solver.format_serial_command(
                    angles, SERIAL_DECIMALS
                )

            except KinematicsError:

                high = scale

            else:

                low = scale

                best = (
                    test_roll,
                    test_pitch,
                    angles,
                    command,
                    True
                )

        return best


def rate_limit_tilt(
    roll,
    pitch,
    previous_roll,
    previous_pitch,
    max_change
):

    delta_roll = roll - previous_roll
    delta_pitch = pitch - previous_pitch

    change = math.hypot(
        delta_roll,
        delta_pitch
    )

    if change > max_change > 0:

        scale = max_change / change

        return (
            previous_roll + delta_roll * scale,
            previous_pitch + delta_pitch * scale
        )

    return roll, pitch


# ==================================================
# ARDUINO SERIAL COMMUNICATION
# ==================================================

def send_command(ser, command, history=None):

    if ser is None:
        return

    ser.write(command)

    if history is not None:

        history.append(
            command.decode(
                "ascii",
                errors="replace"
            ).strip()
        )


def read_arduino_replies(ser, rx_buffer):
    """
    Read complete Arduino response lines without blocking.
    Preserve incomplete lines between reads.
    """

    if ser is None:
        return []

    available = ser.in_waiting

    if available:

        rx_buffer.extend(
            ser.read(min(available, 512))
        )

    if len(rx_buffer) > 2048:

        rx_buffer.clear()

        return [
            "ERR: Arduino reply buffer overflow"
        ]

    replies = []

    while b"\n" in rx_buffer:

        line, _, remaining = rx_buffer.partition(
            b"\n"
        )

        rx_buffer[:] = remaining

        reply = line.decode(
            "utf-8",
            errors="replace"
        ).strip()

        if reply:
            replies.append(reply)

    return replies


def confirms_angles(reply, expected_command):
    """
    Check whether the Arduino acknowledged the neutral command.

    Confirms command acceptance, not physical servo position.
    """

    if not reply.startswith("Angles set:"):
        return False

    try:

        actual = tuple(
            float(v.strip())
            for v in reply.split(":", 1)[1].split(",")
        )

        expected = tuple(
            float(v)
            for v in expected_command.decode(
                "ascii"
            ).strip().split(",")
        )

    except (ValueError, UnicodeDecodeError):
        return False

    return (
        len(actual) == 3
        and len(expected) == 3
        and all(
            abs(a - e) <= 0.051
            for a, e in zip(actual, expected)
        )
    )


# ==================================================
# CAMERA OVERLAY
# ==================================================

def overlay_text(image, lines):

    for i, line in enumerate(lines):

        y = 25 + 25 * i

        cv2.putText(
            image,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.57,
            (0, 0, 0),
            4,
            cv2.LINE_AA
        )

        cv2.putText(
            image,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.57,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )


# ==================================================
# MAIN CONTROLLER
# ==================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--live",
        action="store_true"
    )

    parser.add_argument(
        "--port",
        default=None
    )

    parser.add_argument(
        "--kp",
        type=float,
        default=20.0,
        help="deg of tilt per m of ball offset (default 20)"
    )

    parser.add_argument(
        "--ki",
        type=float,
        default=8.0,
        help="deg per (m*s); removes steady offset (default 8)"
    )

    parser.add_argument(
        "--kd",
        type=float,
        default=25.0,
        help="deg per (m/s); damping (default 25)"
    )

    parser.add_argument(
        "--d-filter",
        type=float,
        default=0.08,
        help="derivative low-pass time constant in s (default 0.08)"
    )

    parser.add_argument(
        "--i-limit",
        type=float,
        default=2.5,
        help="largest tilt the integral term may command, deg"
    )

    parser.add_argument(
        "--max-tilt",
        type=float,
        default=5.0
    )

    parser.add_argument(
        "--tilt-rate",
        type=float,
        default=60.0,
        help="tilt slew limit in deg/s (default 60). A low value "
             "adds lag and makes the ball spiral outward."
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="maximum control rate in Hz; the loop never runs "
             "faster than the camera delivers frames (default 30)"
    )

    parser.add_argument(
        "--trim-roll",
        type=float,
        default=0.0,
        help="constant roll offset in deg to level the plate"
    )

    parser.add_argument(
        "--trim-pitch",
        type=float,
        default=0.0,
        help="constant pitch offset in deg to level the plate"
    )

    parser.add_argument(
        "--lost-timeout",
        type=float,
        default=0.4,
        help="seconds the ball may go undetected before pausing"
    )

    parser.add_argument(
        "--pitch-sign",
        type=int,
        choices=(-1, 1),
        default=1
    )

    parser.add_argument(
        "--roll-sign",
        type=int,
        choices=(-1, 1),
        default=-1
    )

    parser.add_argument(
        "--log",
        action="store_true"
    )

    args = parser.parse_args()

    # Validate settings

    if not all(
        math.isfinite(v) and v >= 0
        for v in (
            args.kp,
            args.ki,
            args.kd
        )
    ):
        parser.error(
            "PID gains must be finite and nonnegative"
        )

    if not (
        math.isfinite(args.max_tilt)
        and 0 < args.max_tilt <= 15.0
    ):
        parser.error(
            "Maximum tilt must be between 0 and 15 degrees "
            "(and no more than max_tilt_deg in kinematics_config.json)"
        )

    if not (
        math.isfinite(args.tilt_rate)
        and 0 < args.tilt_rate <= 500
    ):
        parser.error(
            "Tilt rate must be between 0 and 500 deg/s"
        )

    if not (
        math.isfinite(args.rate)
        and 1 <= args.rate <= 120
    ):
        parser.error(
            "Command rate must be between 1 and 120 Hz"
        )

    if not (
        math.isfinite(args.d_filter)
        and 0 <= args.d_filter <= 1.0
    ):
        parser.error(
            "Derivative filter must be between 0 and 1 s"
        )

    if not (
        math.isfinite(args.i_limit)
        and 0 <= args.i_limit <= args.max_tilt
    ):
        parser.error(
            "Integral limit must be between 0 and the maximum tilt"
        )

    if not (
        math.isfinite(args.lost_timeout)
        and 0 <= args.lost_timeout <= 2.0
    ):
        parser.error(
            "Lost-ball timeout must be between 0 and 2 s"
        )

    if not all(
        math.isfinite(v) and abs(v) <= args.max_tilt
        for v in (args.trim_roll, args.trim_pitch)
    ):
        parser.error(
            "Trim must be smaller than the maximum tilt"
        )

    # ==============================================
    # LOAD CALIBRATION
    # ==============================================

    detector = BallDetector(
        str(ROOT / "stewart_config.json")
    )

    solver = StewartKinematics.from_files()

    if args.max_tilt > solver.max_tilt:

        parser.error(
            "Requested tilt exceeds kinematics_config.json "
            "max_tilt_deg."
        )

    with open(
        ROOT / "stewart_config.json",
        encoding="utf-8"
    ) as file:

        config = json.load(file)

    port = args.port or config["servos"]["port"]

    baud = int(
        config["servos"].get(
            "baud_rate", 9600
        )
    )

    neutral = solver.format_serial_command(
        solver.calculate_servo_angles(
            0.0, 0.0
        ),
        SERIAL_DECIMALS
    )

    print("Neutral command:", neutral)

    pid_x = PID(
        args.kp,
        args.ki,
        args.kd,
        args.d_filter,
        args.i_limit
    )

    pid_y = PID(
        args.kp,
        args.ki,
        args.kd,
        args.d_filter,
        args.i_limit
    )

    # ==============================================
    # CAMERA INITIALIZATION
    # ==============================================

    cap = cv2.VideoCapture(
        detector.camera_index,
        cv2.CAP_DSHOW
    )

    if not cap.isOpened():

        cap.release()

        cap = cv2.VideoCapture(
            detector.camera_index
        )

    if not cap.isOpened():

        raise RuntimeError(
            "Could not open camera"
        )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        detector.frame_width
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        detector.frame_height
    )

    cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1
    )

    # ==============================================
    # CONTROLLER STATE
    # ==============================================

    ser = None

    sent_motor_command = False

    active = False

    prev_update = time.monotonic()
    next_update = prev_update

    last_command = neutral

    recent_commands = deque(maxlen=10)

    rx_buffer = bytearray()

    # Arduino fault and recovery state
    arduino_fault = False
    recovery_pending = False
    recovery_deadline = 0.0

    last_roll = 0.0
    last_pitch = 0.0

    # Anti-windup: integration stops while the tilt is saturated
    saturated = False

    # Short detection dropouts no longer pause balancing
    last_seen = prev_update

    # Diagnostics shown on screen and printed on exit
    loop_dt = 0.0
    active_time = 0.0
    learned_roll = 0.0
    learned_pitch = 0.0

    limit_notice = ""

    status = (
        "MONITOR ONLY"
        if not args.live
        else "PAUSED"
    )

    csv_file = None
    writer = None

    try:

        ok, initial_frame = cap.read()

        if not ok:
            raise RuntimeError(
                "Camera did not return a frame"
            )

        h, w = initial_frame.shape[:2]

        expected_ratio = (
            detector.frame_width /
            detector.frame_height
        )

        if abs((w / h) - expected_ratio) > 0.03:

            raise RuntimeError(
                "Camera aspect ratio differs from calibration. "
                "Recalibrate the camera."
            )

        print("\nStewart Platform PID Controller")

        print(
            "Mode:",
            "LIVE" if args.live else "MONITOR ONLY"
        )

        print(
            f"Kp={args.kp}, Ki={args.ki}, "
            f"Kd={args.kd}"
        )

        print(
            f"Maximum tilt: {args.max_tilt} degrees, "
            f"slew {args.tilt_rate} deg/s, "
            f"up to {args.rate:g} Hz"
        )

        print(
            "SPACE: start/pause | R: neutral/recover | Q: quit"
        )

        # ==========================================
        # CSV LOGGING
        # ==========================================

        if args.log:

            log_path = ROOT / (
                f"pid_run_"
                f"{datetime.now():%Y%m%d_%H%M%S}.csv"
            )

            csv_file = open(
                log_path,
                "w",
                newline="",
                encoding="utf-8"
            )

            writer = csv.writer(csv_file)

            writer.writerow([
                "elapsed_s",
                "found",
                "ball_x_m",
                "ball_y_m",
                "roll_deg",
                "pitch_deg",
                "servo1_deg",
                "servo2_deg",
                "servo3_deg",
                "mode",
                "dt_s",
                "saturated",
                "i_roll_deg",
                "i_pitch_deg"
            ])

            print(
                "Logging to:",
                log_path
            )

        # ==========================================
        # CONNECT TO ARDUINO
        # ==========================================

        if args.live:

            print(
                "\nKeep an accessible physical power cutoff."
            )

            print(
                "Close Arduino Serial Monitor before continuing."
            )

            if input(
                "Type ARM to connect: "
            ).strip() != "ARM":

                print("Not armed.")
                return

            import serial

            ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=0,
                write_timeout=0.5
            )

            time.sleep(2.5)

            ser.reset_input_buffer()

            if input(
                "Type HOME to command neutral: "
            ).strip() != "HOME":

                print("Cancelled.")
                return

            send_command(
                ser,
                neutral,
                recent_commands
            )

            sent_motor_command = True

            print(
                "Neutral position commanded."
            )

            # Confirm the Arduino understood the command. This catches
            # a baud-rate mismatch or the old whole-degree sketch before
            # any balancing is attempted.
            ack_deadline = time.monotonic() + 2.0
            acknowledged = False
            startup_error = ""

            while (
                not acknowledged
                and not startup_error
                and time.monotonic() < ack_deadline
            ):

                for reply in read_arduino_replies(
                    ser,
                    rx_buffer
                ):

                    if confirms_angles(reply, neutral):
                        acknowledged = True

                    elif reply.startswith("ERR:"):
                        startup_error = reply

                time.sleep(0.02)

            if not acknowledged:

                print(
                    "\nArduino did not acknowledge the neutral command"
                    + (
                        f": {startup_error}"
                        if startup_error
                        else " (no valid reply)."
                    )
                )

                print(
                    "Upload servo_controller/servo_controller.ino from "
                    "this folder (decimal angles, 115200 baud) and check "
                    f"that baud_rate in stewart_config.json ({baud}) "
                    "matches Serial.begin() in the sketch."
                )

                return

            print(
                "Arduino acknowledged neutral."
            )

        # ==========================================
        # MAIN FEEDBACK LOOP
        # ==========================================

        started = time.monotonic()

        frame = cv2.resize(
            initial_frame,
            (
                detector.frame_width,
                detector.frame_height
            )
        )

        while True:

            # Detect ball position
            detection = detector.detect_ball(frame)

            if (
                not isinstance(detection, tuple)
                or len(detection) != 5
            ):
                raise RuntimeError(
                    "ball_detection.py must return "
                    "(found, center_px, radius_px, x_m, y_m)"
                )

            (
                found,
                center_px,
                ball_radius_px,
                x_m,
                y_m
            ) = detection

            now = time.monotonic()

            roll = 0.0
            pitch = 0.0

            commanded_angles = solver.neutral_servo

            # ======================================
            # LOST BALL HANDLING
            # ======================================

            if found:
                last_seen = now

            # A single missed detection used to pause balancing and send
            # the plate to neutral. Hold the last command instead, and
            # only pause when the ball stays missing.
            if (
                active
                and not found
                and now - last_seen > args.lost_timeout
            ):

                active = False
                status = "BALL LOST: PAUSED"

                pid_x.reset()
                pid_y.reset()

                if ser is not None:

                    send_command(
                        ser,
                        neutral,
                        recent_commands
                    )

                    last_command = neutral

                last_roll = 0.0
                last_pitch = 0.0

                limit_notice = ""

                print(status)

            # ======================================
            # PID CALCULATION
            # ======================================

            if found and now >= next_update:

                dt = now - prev_update

                prev_update = now

                # The 0.75 factor stops timing jitter from skipping
                # every other camera frame when --rate equals the
                # camera frame rate.
                next_update = (
                    now + 0.75 / args.rate
                )

                loop_dt += 0.1 * (
                    min(dt, 0.5) - loop_dt
                )

                error_x = -float(x_m)
                error_y = -float(y_m)

                if active or not args.live:

                    # Integrate only while balancing and not saturated
                    integrate = active and not saturated

                    # X error controls pitch
                    # (+pitch accelerates the ball toward +x)
                    pitch = (
                        args.pitch_sign *
                        pid_x.update(error_x, dt, integrate)
                        + args.trim_pitch
                    )

                    # Y error controls roll
                    # (+roll accelerates the ball toward -y)
                    roll = (
                        args.roll_sign *
                        pid_y.update(error_y, dt, integrate)
                        + args.trim_roll
                    )

                    wanted_tilt = math.hypot(roll, pitch)

                    # Maximum combined tilt
                    roll, pitch = limit_vector(
                        roll,
                        pitch,
                        args.max_tilt
                    )

                    # Rate limiting
                    roll, pitch = rate_limit_tilt(
                        roll,
                        pitch,
                        last_roll,
                        last_pitch,
                        args.tilt_rate *
                        max(0.001, min(dt, 0.2))
                    )

                    try:

                        (
                            roll,
                            pitch,
                            commanded_angles,
                            command,
                            clipped
                        ) = reachable_tilt(
                            solver,
                            roll,
                            pitch
                        )

                        if clipped:

                            limit_notice = (
                                "SERVO LIMIT: reduced tilt"
                            )

                        else:

                            limit_notice = ""

                        saturated = (
                            clipped
                            or wanted_tilt > args.max_tilt
                        )

                        last_roll = roll
                        last_pitch = pitch

                        if active:

                            active_time += min(dt, 0.2)

                            # Slow average of the tilt needed to hold
                            # the ball still (trim + integral term).
                            blend = min(dt, 0.2) / 3.0

                            learned_roll += blend * (
                                args.trim_roll
                                + args.roll_sign * pid_y.integral_term
                                - learned_roll
                            )

                            learned_pitch += blend * (
                                args.trim_pitch
                                + args.pitch_sign * pid_x.integral_term
                                - learned_pitch
                            )

                        if (
                            active
                            and ser is not None
                            and command != last_command
                        ):

                            send_command(
                                ser,
                                command,
                                recent_commands
                            )

                            last_command = command

                    except (
                        KinematicsError,
                        ValueError
                    ) as exc:

                        status = (
                            f"IK REJECTED: {exc}"
                        )

                        if active:

                            active = False

                            if ser is not None:

                                send_command(
                                    ser,
                                    neutral,
                                    recent_commands
                                )

                                last_command = neutral

                        last_roll = 0.0
                        last_pitch = 0.0

                        limit_notice = ""

                        pid_x.reset()
                        pid_y.reset()

                        print(status)

                else:

                    pid_x.reset()
                    pid_y.reset()

                    last_roll = 0.0
                    last_pitch = 0.0

                    saturated = False

                    limit_notice = ""

                if writer is not None:

                    writer.writerow([
                        round(now - started, 3),
                        int(found),
                        round(x_m, 5),
                        round(y_m, 5),
                        round(roll, 4),
                        round(pitch, 4),
                        *[
                            round(a, 3)
                            for a in commanded_angles
                        ],
                        (
                            "ACTIVE"
                            if active
                            else status
                        ),
                        round(dt, 4),
                        int(saturated),
                        round(
                            args.roll_sign * pid_y.integral_term,
                            4
                        ),
                        round(
                            args.pitch_sign * pid_x.integral_term,
                            4
                        )
                    ])

            # ======================================
            # ARDUINO ERROR / RECOVERY HANDLING
            # ======================================

            if ser is not None:

                replies = read_arduino_replies(
                    ser,
                    rx_buffer
                )

                for reply in replies:

                    # Arduino rejected a command
                    if reply.startswith("ERR:"):

                        print(
                            "[ARDUINO ERROR]",
                            reply
                        )

                        print(
                            "[RECENT TX]",
                            list(recent_commands)
                        )

                        first_fault = not arduino_fault

                        arduino_fault = True
                        recovery_pending = False

                        active = False

                        status = (
                            "ARDUINO ERROR: PAUSED - PRESS R"
                        )

                        limit_notice = reply[:100]

                        pid_x.reset()
                        pid_y.reset()

                        last_roll = 0.0
                        last_pitch = 0.0

                        # Attempt neutral once after initial error
                        if first_fault:

                            try:

                                send_command(
                                    ser,
                                    neutral,
                                    recent_commands
                                )

                                last_command = neutral

                            except Exception as exc:

                                print(
                                    "[SERIAL ERROR]",
                                    exc
                                )

                                status = (
                                    "SERIAL FAULT: STOP MOTORS"
                                )

                        print(
                            "Press R to retry neutral, "
                            "then SPACE to resume."
                        )

                    # Arduino acknowledged neutral recovery
                    elif (
                        recovery_pending
                        and confirms_angles(reply, neutral)
                    ):

                        recovery_pending = False
                        arduino_fault = False

                        active = False

                        status = (
                            "RECOVERED: PRESS SPACE TO START"
                        )

                        limit_notice = (
                            "Neutral command acknowledged"
                        )

                        pid_x.reset()
                        pid_y.reset()

                        last_roll = 0.0
                        last_pitch = 0.0

                        print(
                            "[RECOVERED] Arduino accepted neutral."
                        )

                        print(
                            "Check physical position, "
                            "then press SPACE."
                        )

                    elif reply.startswith("READY:"):

                        print(
                            "Arduino:",
                            reply
                        )

                # Recovery timeout
                if (
                    recovery_pending
                    and time.monotonic() >= recovery_deadline
                ):

                    recovery_pending = False
                    arduino_fault = True

                    active = False

                    status = (
                        "RECOVERY TIMEOUT: PRESS R"
                    )

                    limit_notice = (
                        "No neutral acknowledgment"
                    )

                    print(
                        "[RECOVERY TIMEOUT] "
                        "Press R to retry."
                    )

            # ======================================
            # CAMERA DISPLAY
            # ======================================

            # Reuse this frame's detection instead of detecting twice
            display, _, _, _ = (
                detector.draw_detection(frame, detection)
            )

            if center_px is not None:

                cv2.circle(
                    display,
                    center_px,
                    max(3, int(ball_radius_px)),
                    (0, 255, 255),
                    2
                )

            cv2.circle(
                display,
                detector.platform_center,
                7,
                (255, 0, 255),
                2
            )

            mode = (
                "BALANCING"
                if active
                else status
            )

            if not args.live:

                mode = (
                    "MONITOR ONLY (NO SERVO COMMANDS)"
                )

            overlay_text(
                display,
                [
                    mode,

                    (
                        f"Ball: x={x_m:+.3f} m "
                        f"y={y_m:+.3f} m"
                        if found
                        else "BALL NOT DETECTED"
                    ),

                    (
                        f"Roll={roll:+.2f} deg "
                        f"Pitch={pitch:+.2f} deg "
                        f"(max {args.max_tilt:g})"
                    ),

                    (
                        f"Loop {1.0 / loop_dt:4.1f} Hz  "
                        f"I-term roll={args.roll_sign * pid_y.integral_term:+.2f} "
                        f"pitch={args.pitch_sign * pid_x.integral_term:+.2f}"
                        if loop_dt > 0
                        else "Loop -- Hz"
                    ),

                    (
                        limit_notice
                        or "Servo limits enforced"
                    ),

                    (
                        "SPACE: start/pause "
                        "R: neutral/recover Q: quit"
                    )
                ]
            )

            cv2.imshow(
                "Stewart Platform PID Control",
                display
            )

            key = cv2.waitKey(1) & 0xFF

            if cv2.getWindowProperty(
                "Stewart Platform PID Control",
                cv2.WND_PROP_VISIBLE
            ) < 1:
                break

            if key in (ord("q"), 27):
                break

            # ======================================
            # R KEY: NEUTRAL / ERROR RECOVERY
            # ======================================

            if key == ord("r") and args.live:

                active = False

                pid_x.reset()
                pid_y.reset()

                last_roll = 0.0
                last_pitch = 0.0

                if arduino_fault:

                    try:

                        # Remove stale serial replies
                        ser.reset_input_buffer()
                        rx_buffer.clear()

                        # Send calibrated neutral
                        send_command(
                            ser,
                            neutral,
                            recent_commands
                        )

                        last_command = neutral

                        # Wait for matching Arduino ACK
                        recovery_pending = True

                        recovery_deadline = (
                            time.monotonic() + 2.0
                        )

                        status = (
                            "RECOVERING: WAITING FOR ACK"
                        )

                        limit_notice = (
                            "Waiting for Arduino neutral ACK"
                        )

                        print(
                            "[RECOVERY] Neutral sent."
                        )

                    except Exception as exc:

                        recovery_pending = False

                        status = (
                            "SERIAL FAULT: STOP MOTORS"
                        )

                        limit_notice = str(exc)[:100]

                        print(
                            "[RECOVERY FAILED]",
                            exc
                        )

                else:

                    status = "PAUSED (neutral)"

                    if ser is not None:

                        send_command(
                            ser,
                            neutral,
                            recent_commands
                        )

                        last_command = neutral

                    limit_notice = ""

            # ======================================
            # SPACE KEY: START / PAUSE
            # ======================================

            elif key == ord(" ") and args.live:

                if arduino_fault or recovery_pending:

                    print(
                        "Controller paused after Arduino error."
                    )

                    print(
                        "Press R, wait for neutral ACK, "
                        "then SPACE."
                    )

                elif active:

                    active = False

                    status = "PAUSED (neutral)"

                    pid_x.reset()
                    pid_y.reset()

                    if ser is not None:

                        send_command(
                            ser,
                            neutral,
                            recent_commands
                        )

                        last_command = neutral

                    last_roll = 0.0
                    last_pitch = 0.0

                    limit_notice = ""

                elif found:

                    pid_x.reset()
                    pid_y.reset()

                    prev_update = time.monotonic()
                    next_update = prev_update

                    active = True

                    status = "BALANCING"

                    print(
                        "Balancing enabled."
                    )

                else:

                    print(
                        "Cannot start: ball not detected."
                    )

            # ======================================
            # READ NEXT CAMERA FRAME
            # ======================================

            ok, next_frame = cap.read()

            if not ok:

                if active:

                    active = False

                    status = (
                        "CAMERA FAILURE: PAUSED"
                    )

                    pid_x.reset()
                    pid_y.reset()

                    if ser is not None:

                        send_command(
                            ser,
                            neutral,
                            recent_commands
                        )

                        last_command = neutral

                    last_roll = 0.0
                    last_pitch = 0.0

                print(
                    "Camera read failed."
                )

                break

            if next_frame.shape[:2] != (h, w):

                raise RuntimeError(
                    "Camera frame dimensions changed."
                )

            frame = cv2.resize(
                next_frame,
                (
                    detector.frame_width,
                    detector.frame_height
                )
            )

    except KeyboardInterrupt:

        print("Interrupted.")

    except Exception as exc:

        print(
            "[CONTROLLER STOPPED]",
            exc
        )

        print(
            "Use physical power cutoff if necessary."
        )

    finally:

        if ser is not None:

            if sent_motor_command:

                try:

                    send_command(
                        ser,
                        neutral,
                        recent_commands
                    )

                    time.sleep(0.1)

                except Exception as exc:

                    print(
                        "Failed to send neutral:",
                        exc
                    )

            ser.close()

        cap.release()
        cv2.destroyAllWindows()

        if csv_file is not None:
            csv_file.close()

        if active_time > 15.0:

            print(
                f"Tilt needed to hold the ball still, averaged over "
                f"the last seconds of balancing: "
                f"roll {learned_roll:+.2f} deg, "
                f"pitch {learned_pitch:+.2f} deg."
            )

            print(
                "If these are larger than about 0.3 deg the plate is "
                "not level at neutral. Start next time with "
                f"--trim-roll {learned_roll:.2f} "
                f"--trim-pitch {learned_pitch:.2f} "
                "(or re-level the neutral angles)."
            )

        print(
            "Controller stopped."
        )


if __name__ == "__main__":
    main()
