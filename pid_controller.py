
"""Camera-based PID controller for a 3-servo Stewart platform.

Uses:
    ball_detection.py
    kinematics.py
    stewart_config.json
    kinematics_config.json

Arduino: PCA9685 channels 0, 3, 7.
Serial: comma-separated angles at 9600 baud.

Usage:
    python pid_controller.py
    python pid_controller.py --live --kp 10 --max-tilt 8 --log

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


# ==================================================
# PID CONTROLLER
# ==================================================

class PID:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def update(self, error, dt):

        dt = max(0.001, min(dt, 0.2))

        if self.previous_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (
                error - self.previous_error
            ) / dt

        self.previous_error = error

        # Filter derivative noise
        tau = 0.15
        alpha = dt / (tau + dt)

        self.filtered_derivative += alpha * (
            raw_derivative - self.filtered_derivative
        )

        # Integral anti-windup
        self.integral += error * dt

        self.integral = max(
            -0.1,
            min(0.1, self.integral)
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
            angles
        )

        return roll, pitch, angles, command, False

    except KinematicsError:

        home_angles = solver.calculate_servo_angles(
            0.0, 0.0
        )

        home_command = solver.format_serial_command(
            home_angles
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
                    angles
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
            int(v.strip())
            for v in reply.split(":", 1)[1].split(",")
        )

        expected = tuple(
            int(v)
            for v in expected_command.decode(
                "ascii"
            ).strip().split(",")
        )

    except (ValueError, UnicodeDecodeError):
        return False

    return (
        len(actual) == 3
        and actual == expected
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
        default=40.0
    )

    parser.add_argument(
        "--ki",
        type=float,
        default=0.0
    )

    parser.add_argument(
        "--kd",
        type=float,
        default=0.0
    )

    parser.add_argument(
        "--max-tilt",
        type=float,
        default=8.0
    )

    parser.add_argument(
        "--tilt-rate",
        type=float,
        default=12.0
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=12.0
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
        and 0 < args.max_tilt <= 5.0
    ):
        parser.error(
            "Maximum tilt must be between 0 and 8 degrees"
        )

    if not (
        math.isfinite(args.tilt_rate)
        and 0 < args.tilt_rate <= 30
    ):
        parser.error(
            "Invalid tilt rate"
        )

    if not (
        math.isfinite(args.rate)
        and 1 <= args.rate <= 15
    ):
        parser.error(
            "Command rate must be between 1 and 15 Hz"
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
        )
    )

    print("Neutral command:", neutral)

    pid_x = PID(
        args.kp,
        args.ki,
        args.kd
    )

    pid_y = PID(
        args.kp,
        args.ki,
        args.kd
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
            f"Maximum tilt: {args.max_tilt} degrees"
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
                "mode"
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

            if active and not found:

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

                next_update = (
                    now + 1.0 / args.rate
                )

                error_x = -float(x_m)
                error_y = -float(y_m)

                if active or not args.live:

                    # X error controls pitch
                    pitch = (
                        args.pitch_sign *
                        pid_x.update(error_x, dt)
                    )

                    # Y error controls roll
                    roll = (
                        args.roll_sign *
                        pid_y.update(error_y, dt)
                    )

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

                        last_roll = roll
                        last_pitch = pitch

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

            display, _, _, _ = (
                detector.draw_detection(frame)
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

        print(
            "Controller stopped."
        )


if __name__ == "__main__":
    main()
