"""Trajectory tracking demo for the 3-servo Stewart platform.

Makes the ball trace a shape (circle, figure 8, star, square, triangle)
instead of sitting at the centre. See trajectory_theory.html for the theory.

Uses the same files as pid_controller.py:
    ball_detection.py, kinematics.py, pid_controller.py,
    stewart_config.json, kinematics_config.json

Usage:
    python trajectory_demo.py --sim                    # simulated ball, no hardware
    python trajectory_demo.py                          # camera only, no servo commands
    python trajectory_demo.py --live --log             # real platform
    python trajectory_demo.py --live --shape star --size 0.06 --period 15

Controls:
    SPACE = Start / pause
    1-5   = circle / figure 8 / star / square / triangle
    F     = Feedforward on / off (watch the tracking error change)
    R     = Return to neutral
    Q/ESC = Quit

How it works, in one paragraph: a planner turns the shape into a reference
position for every instant, ref(t). The plate tilt is the sum of two parts.
Feedforward is the tilt that would give the ball the acceleration of the
reference, tilt = acceleration / plant gain, evaluated one dead time ahead.
Feedback is the same PID as pid_controller.py, acting on (ref - ball)
instead of (0 - ball), and it cleans up whatever the feedforward gets wrong.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import time

from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from ball_detection import BallDetector
from kinematics import KinematicsError, StewartKinematics
from pid_controller import (
    PID,
    SERIAL_DECIMALS,
    confirms_angles,
    limit_vector,
    overlay_text,
    rate_limit_tilt,
    reachable_tilt,
    read_arduino_replies,
    send_command,
)


ROOT = Path(__file__).resolve().parent

WINDOW = "Stewart Platform Trajectory Demo"

SHAPES = ["circle", "figure8", "star", "square", "triangle"]

# Seconds per lap. Slow enough that the feedforward tilt stays near 1 deg.
DEFAULT_PERIOD = {
    "circle": 8.0,
    "figure8": 12.0,
    "star": 15.0,
    "square": 12.0,
    "triangle": 10.0,
}

# Feedforward may use at most this share of the tilt limit; the rest is
# kept for feedback.
FEEDFORWARD_SHARE = 0.6


# ==================================================
# TRAJECTORY PLANNING
# ==================================================

def min_jerk(u):
    """Smooth 0-to-1 time scaling with zero speed and zero acceleration
    at both ends (quintic polynomial 10u^3 - 15u^4 + 6u^5)."""

    u = min(1.0, max(0.0, u))

    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def polygon_vertices(count, step, radius, start_deg):
    """Points on a circle, visited every `step`-th one (step 2 of 5 = star)."""

    return [
        (
            radius * math.cos(math.radians(start_deg + 360.0 * step * k / count)),
            radius * math.sin(math.radians(start_deg + 360.0 * step * k / count)),
        )
        for k in range(count)
    ]


class Trajectory:
    """A closed shape plus a timing law: position(t) in metres, one lap
    every `period` seconds."""

    def __init__(self, shape, size, period=None):

        if shape not in SHAPES:
            raise ValueError(f"Unknown shape: {shape}")

        self.shape = shape
        self.size = size
        self.period = period or DEFAULT_PERIOD[shape]

        # Shapes with corners are lists of waypoints. The ball stops at each
        # corner, because turning a corner at speed needs infinite acceleration.
        self.vertices = None

        if shape == "star":
            self.vertices = polygon_vertices(5, 2, size, 90.0)

        elif shape == "square":
            self.vertices = polygon_vertices(4, 1, size, 45.0)

        elif shape == "triangle":
            self.vertices = polygon_vertices(3, 1, size, 90.0)

    def position(self, t):

        t = t % self.period

        if self.vertices is not None:

            count = len(self.vertices)
            progress = t / (self.period / count)
            index = int(progress) % count

            ax, ay = self.vertices[index]
            bx, by = self.vertices[(index + 1) % count]

            s = min_jerk(progress - int(progress))

            return ax + (bx - ax) * s, ay + (by - ay) * s

        angle = 2.0 * math.pi * t / self.period

        if self.shape == "circle":
            return self.size * math.cos(angle), self.size * math.sin(angle)

        # Figure 8 (lemniscate of Gerono)
        return (
            self.size * math.sin(angle),
            self.size * math.sin(angle) * math.cos(angle),
        )

    def peak_acceleration(self, samples=2000):
        """Largest acceleration the reference asks for, in m/s^2."""

        h = self.period / samples
        peak = 0.0

        for k in range(samples):

            t = k * h

            x0, y0 = self.position(t - h)
            x1, y1 = self.position(t)
            x2, y2 = self.position(t + h)

            peak = max(
                peak,
                math.hypot(x0 - 2 * x1 + x2, y0 - 2 * y1 + y2) / (h * h),
            )

        return peak

    def outline(self, samples=400):

        return [
            self.position(self.period * k / samples)
            for k in range(samples + 1)
        ]


class Plan:
    """What the ball should do after SPACE is pressed:
    glide from where it is to the start of the shape, settle, then lap."""

    APPROACH_S = 3.0
    SETTLE_S = 1.5
    RAMP_S = 2.0

    def __init__(self, trajectory, start_xy, start_time):

        self.trajectory = trajectory
        self.start_xy = start_xy
        self.start_time = start_time
        self.entry_xy = trajectory.position(0.0)

    def lap_time(self, t):
        """Time along the shape. Speed ramps smoothly from 0 to 1 over
        RAMP_S so a shape that starts at speed (circle) starts gently."""

        tau = t - self.start_time - self.APPROACH_S - self.SETTLE_S

        if tau <= 0.0:
            return 0.0

        if tau < self.RAMP_S:

            u = tau / self.RAMP_S

            return self.RAMP_S * (u ** 3 - 0.5 * u ** 4)

        return tau - 0.5 * self.RAMP_S

    def phase(self, t):

        tau = t - self.start_time

        if tau < self.APPROACH_S:
            return "APPROACH"

        if tau < self.APPROACH_S + self.SETTLE_S:
            return "SETTLE"

        return "TRACKING"

    def laps(self, t):

        return self.lap_time(t) / self.trajectory.period

    def position(self, t):

        tau = t - self.start_time

        if tau < self.APPROACH_S:

            s = min_jerk(tau / self.APPROACH_S)

            return (
                self.start_xy[0] + (self.entry_xy[0] - self.start_xy[0]) * s,
                self.start_xy[1] + (self.entry_xy[1] - self.start_xy[1]) * s,
            )

        return self.trajectory.position(self.lap_time(t))

    def acceleration(self, t, h=0.02):
        """Second derivative of the reference, by central differences."""

        x0, y0 = self.position(t - h)
        x1, y1 = self.position(t)
        x2, y2 = self.position(t + h)

        return (
            (x0 - 2.0 * x1 + x2) / (h * h),
            (y0 - 2.0 * y1 + y2) / (h * h),
        )


# ==================================================
# TRACKING CONTROLLER
# ==================================================

class Tracker:
    """Feedforward + PID feedback. Output is a reachable roll/pitch."""

    def __init__(self, args, solver):

        self.args = args
        self.solver = solver

        self.pid_x = PID(args.kp, args.ki, args.kd, args.d_filter, args.i_limit)
        self.pid_y = PID(args.kp, args.ki, args.kd, args.d_filter, args.i_limit)

        self.feedforward = not args.no_ff

        self.reset()

    def reset(self):

        self.pid_x.reset()
        self.pid_y.reset()

        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.saturated = False

    def update(self, plan, t, x, y, dt):

        args = self.args

        ref_x, ref_y = plan.position(t)

        # Feedforward: tilt (deg) that produces the reference acceleration.
        # Looking one dead time ahead cancels most of the loop delay.
        ff_x = 0.0
        ff_y = 0.0

        if self.feedforward:

            acc_x, acc_y = plan.acceleration(t + args.delay)

            ff_x = acc_x / args.plant_gain
            ff_y = acc_y / args.plant_gain

        # Feedback on the tracking error
        integrate = not self.saturated

        push_x = self.pid_x.update(ref_x - x, dt, integrate) + ff_x
        push_y = self.pid_y.update(ref_y - y, dt, integrate) + ff_y

        # +pitch accelerates the ball toward +x, +roll toward -y
        pitch = args.pitch_sign * push_x + args.trim_pitch
        roll = args.roll_sign * push_y + args.trim_roll

        wanted = math.hypot(roll, pitch)

        roll, pitch = limit_vector(roll, pitch, args.max_tilt)

        roll, pitch = rate_limit_tilt(
            roll,
            pitch,
            self.last_roll,
            self.last_pitch,
            args.tilt_rate * max(0.001, min(dt, 0.2)),
        )

        roll, pitch, angles, command, clipped = reachable_tilt(
            self.solver, roll, pitch
        )

        self.saturated = clipped or wanted > args.max_tilt

        self.last_roll = roll
        self.last_pitch = pitch

        return {
            "roll": roll,
            "pitch": pitch,
            "angles": angles,
            "command": command,
            "ref": (ref_x, ref_y),
            "error_m": math.hypot(ref_x - x, ref_y - y),
            "ff_tilt": math.hypot(ff_x, ff_y),
        }


# ==================================================
# SIMULATED BALL (--sim)
# ==================================================

class SimulatedBall:
    """Rolling ball with the dead time and gain measured on the real
    platform (CHANGES.md). The gain is deliberately 10% off the value the
    controller assumes, and the plate is slightly off level."""

    def __init__(self, gain, delay, plate_radius):

        self.gain = gain * 0.9
        self.delay = delay
        self.plate_radius = plate_radius

        self.bias = (0.02, -0.015)

        self.x, self.y = 0.045, -0.03
        self.vx, self.vy = 0.0, 0.0

        self.history = deque()

    def step(self, t, dt, push_x, push_y):

        self.history.append((t, push_x, push_y))

        while len(self.history) > 1 and self.history[1][0] <= t - self.delay:
            self.history.popleft()

        delayed_x, delayed_y = 0.0, 0.0

        if self.history[0][0] <= t - self.delay:
            _, delayed_x, delayed_y = self.history[0]

        dt = min(dt, 0.1)

        self.vx += (self.gain * delayed_x + self.bias[0]) * dt
        self.vy += (self.gain * delayed_y + self.bias[1]) * dt

        self.x += self.vx * dt
        self.y += self.vy * dt

        distance = math.hypot(self.x, self.y)
        rim = self.plate_radius - 0.02

        if distance > rim:

            self.x *= rim / distance
            self.y *= rim / distance
            self.vx = 0.0
            self.vy = 0.0

    def measure(self):

        return (
            self.x + random.gauss(0.0, 0.0003),
            self.y + random.gauss(0.0, 0.0003),
        )


# ==================================================
# HELPERS
# ==================================================

def check_feasible(trajectory, args):
    """Return (ok, message). The shape is refused when its feedforward
    alone would use more than FEEDFORWARD_SHARE of the tilt limit."""

    peak = trajectory.peak_acceleration()
    tilt = peak / args.plant_gain
    budget = FEEDFORWARD_SHARE * args.max_tilt

    message = (
        f"{trajectory.shape}: size {trajectory.size * 1000:.0f} mm, "
        f"{trajectory.period:g} s per lap, peak acceleration "
        f"{peak:.3f} m/s^2 = {tilt:.2f} deg of tilt "
        f"(budget {budget:.1f} deg)"
    )

    if tilt > budget:

        # Acceleration scales with 1 / period^2
        needed = trajectory.period * math.sqrt(tilt / budget)

        return False, (
            message
            + f"\nToo fast for this plate. Use --period {math.ceil(needed)} "
            "or a smaller --size."
        )

    return True, message


def parse_args():

    parser = argparse.ArgumentParser(
        description="Trajectory tracking demo (see trajectory_theory.html)"
    )

    parser.add_argument("--live", action="store_true",
                        help="send servo commands to the Arduino")
    parser.add_argument("--sim", action="store_true",
                        help="simulated ball; no camera or Arduino needed")
    parser.add_argument("--port", default=None)

    parser.add_argument("--shape", choices=SHAPES, default="circle")
    parser.add_argument("--size", type=float, default=0.06,
                        help="shape radius in m (default 0.06)")
    parser.add_argument("--period", type=float, default=None,
                        help="seconds per lap (default depends on the shape)")

    parser.add_argument("--no-ff", action="store_true",
                        help="start with feedforward off (F toggles it)")
    parser.add_argument("--plant-gain", type=float, default=0.070,
                        help="ball acceleration in m/s^2 per deg of tilt "
                             "(default 0.070, measured by analyze_log.py)")
    parser.add_argument("--delay", type=float, default=0.20,
                        help="loop dead time in s (default 0.20)")

    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--ki", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=25.0)
    parser.add_argument("--d-filter", type=float, default=0.08)
    parser.add_argument("--i-limit", type=float, default=2.5)
    parser.add_argument("--max-tilt", type=float, default=5.0)
    parser.add_argument("--tilt-rate", type=float, default=60.0)
    parser.add_argument("--trim-roll", type=float, default=0.0)
    parser.add_argument("--trim-pitch", type=float, default=0.0)
    parser.add_argument("--lost-timeout", type=float, default=0.4)
    parser.add_argument("--pitch-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--roll-sign", type=int, choices=[-1, 1], default=-1)

    parser.add_argument("--log", action="store_true",
                        help="write traj_run_<time>.csv")

    args = parser.parse_args()

    if args.live and args.sim:
        parser.error("Use --live or --sim, not both")

    if not 0.01 <= args.size <= 0.10:
        parser.error("Size must be between 0.01 and 0.10 m")

    if args.period is not None and not 2.0 <= args.period <= 120.0:
        parser.error("Period must be between 2 and 120 s")

    if not 0.01 <= args.plant_gain <= 0.5:
        parser.error("Plant gain must be between 0.01 and 0.5 m/s^2 per deg")

    if not 0.0 <= args.delay <= 1.0:
        parser.error("Delay must be between 0 and 1 s")

    if not 0 < args.max_tilt <= 15.0:
        parser.error("Maximum tilt must be between 0 and 15 degrees")

    if not 0 <= args.i_limit <= args.max_tilt:
        parser.error("Integral limit must be between 0 and the maximum tilt")

    return args


def open_camera(detector):

    cap = cv2.VideoCapture(detector.camera_index, cv2.CAP_DSHOW)

    if not cap.isOpened():

        cap.release()
        cap = cv2.VideoCapture(detector.camera_index)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, detector.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, detector.frame_height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    return cap


def connect_arduino(port, baud, neutral, history, rx_buffer):
    """ARM / HOME prompts, then wait for the Arduino to acknowledge
    neutral. Returns the open port, or None if the user cancelled."""

    print("\nKeep an accessible physical power cutoff.")
    print("Close Arduino Serial Monitor before continuing.")

    if input("Type ARM to connect: ").strip() != "ARM":
        print("Not armed.")
        return None

    import serial

    ser = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0.5)

    time.sleep(2.5)
    ser.reset_input_buffer()

    if input("Type HOME to command neutral: ").strip() != "HOME":
        print("Cancelled.")
        ser.close()
        return None

    send_command(ser, neutral, history)

    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:

        for reply in read_arduino_replies(ser, rx_buffer):

            if confirms_angles(reply, neutral):
                print("Arduino acknowledged neutral.")
                return ser

            if reply.startswith("ERR:"):
                deadline = 0.0
                print("Arduino replied:", reply)

        time.sleep(0.02)

    print(
        "Arduino did not acknowledge neutral. Upload "
        "servo_controller/servo_controller.ino from this folder and check "
        "the baud rate in stewart_config.json."
    )

    try:
        send_command(ser, neutral, history)
        time.sleep(0.1)
    finally:
        ser.close()

    return None


# ==================================================
# MAIN
# ==================================================

def main():

    args = parse_args()

    detector = BallDetector(str(ROOT / "stewart_config.json"))
    solver = StewartKinematics.from_files()

    if args.max_tilt > solver.max_tilt:
        raise SystemExit(
            "Requested tilt exceeds kinematics_config.json max_tilt_deg."
        )

    with open(ROOT / "stewart_config.json", encoding="utf-8") as file:
        config = json.load(file)

    port = args.port or config["servos"]["port"]
    baud = int(config["servos"].get("baud_rate", 9600))

    neutral = solver.format_serial_command(
        solver.calculate_servo_angles(0.0, 0.0), SERIAL_DECIMALS
    )

    trajectory = Trajectory(args.shape, args.size, args.period)

    ok, message = check_feasible(trajectory, args)
    print("\n" + message)

    if not ok:
        raise SystemExit(1)

    tracker = Tracker(args, solver)

    scale = detector.pixel_to_meter_ratio
    centre_x, centre_y = detector.platform_center

    def to_pixels(x_m, y_m):

        return (
            int(round(centre_x + x_m / scale)),
            int(round(centre_y - y_m / scale)),
        )

    def outline_pixels(shape_trajectory):

        return np.array(
            [to_pixels(x, y) for x, y in shape_trajectory.outline()],
            dtype=np.int32,
        ).reshape(-1, 1, 2)

    outline = outline_pixels(trajectory)

    # State
    ser = None
    cap = None
    simulated = None
    csv_file = None
    writer = None

    recent_commands = deque(maxlen=10)
    rx_buffer = bytearray()
    last_command = neutral

    active = False
    plan = None
    status = "PRESS SPACE TO START"

    trail = deque(maxlen=600)
    lap_errors = deque()

    # Tracking error totals, feedforward off [0] and on [1]
    error_sum = [0.0, 0.0]
    error_count = [0, 0]

    result = None
    roll = 0.0
    pitch = 0.0
    loop_dt = 0.0

    def stop(reason):

        nonlocal active, plan, status, last_command, roll, pitch, result

        active = False
        plan = None
        status = reason
        result = None
        roll = 0.0
        pitch = 0.0

        tracker.reset()
        lap_errors.clear()

        if ser is not None:
            send_command(ser, neutral, recent_commands)
            last_command = neutral

    try:

        if args.sim:

            simulated = SimulatedBall(
                args.plant_gain,
                args.delay,
                detector.platform_radius_px * scale,
            )

            blank = np.full(
                (detector.frame_height, detector.frame_width, 3),
                (60, 60, 60),
                dtype=np.uint8,
            )

            cv2.circle(
                blank,
                detector.platform_center,
                detector.platform_radius_px,
                (205, 205, 205),
                -1,
            )

        else:

            cap = open_camera(detector)

            ok, frame = cap.read()

            if not ok:
                raise RuntimeError("Camera did not return a frame")

        mode_name = "SIMULATION" if args.sim else "LIVE" if args.live else "CAMERA ONLY"

        print("Mode:", mode_name)
        print(f"Kp={args.kp}, Ki={args.ki}, Kd={args.kd}, "
              f"feedforward {'on' if tracker.feedforward else 'off'}")
        print("SPACE: start/pause | 1-5: shape | F: feedforward | "
              "R: neutral | Q: quit")

        if args.log:

            log_path = ROOT / f"traj_run_{datetime.now():%Y%m%d_%H%M%S}.csv"

            csv_file = open(log_path, "w", newline="", encoding="utf-8")
            writer = csv.writer(csv_file)

            writer.writerow([
                "elapsed_s", "shape", "phase", "feedforward",
                "ref_x_m", "ref_y_m", "ball_x_m", "ball_y_m", "error_mm",
                "roll_deg", "pitch_deg", "ff_tilt_deg", "dt_s",
            ])

            print("Logging to:", log_path)

        if args.live:

            ser = connect_arduino(
                port, baud, neutral, recent_commands, rx_buffer
            )

            if ser is None:
                return

        started = time.monotonic()
        previous = started
        last_seen = started

        while True:

            now = time.monotonic()
            dt = now - previous
            previous = now

            loop_dt += 0.1 * (min(dt, 0.5) - loop_dt)

            # ----- Where is the ball? -----

            if simulated is not None:

                if active:

                    simulated.step(
                        now,
                        dt,
                        args.pitch_sign * pitch,
                        args.roll_sign * roll,
                    )

                frame = blank.copy()

                x_m, y_m = simulated.measure()

                detection = (
                    True, to_pixels(x_m, y_m), 0.02 / scale, x_m, y_m
                )

                cv2.circle(
                    frame, detection[1], int(detection[2]), (0, 140, 255), -1
                )

            else:

                frame = cv2.resize(
                    frame, (detector.frame_width, detector.frame_height)
                )

                detection = detector.detect_ball(frame)

            found, centre_px, _, x_m, y_m = detection

            if found:
                last_seen = now

            elif active and now - last_seen > args.lost_timeout:
                stop("BALL LOST: PAUSED")
                print(status)

            # ----- Control -----

            if active and found:

                try:
                    result = tracker.update(plan, now, x_m, y_m, dt)

                except (KinematicsError, ValueError) as exc:
                    stop(f"IK REJECTED: {exc}")
                    print(status)

                else:

                    roll = result["roll"]
                    pitch = result["pitch"]

                    if ser is not None and result["command"] != last_command:

                        send_command(ser, result["command"], recent_commands)
                        last_command = result["command"]

                    phase = plan.phase(now)

                    if phase == "TRACKING":

                        lap_errors.append((now, result["error_m"] ** 2))

                        index = int(tracker.feedforward)
                        error_sum[index] += result["error_m"] ** 2
                        error_count[index] += 1

                    while (
                        lap_errors
                        and lap_errors[0][0] < now - trajectory.period
                    ):
                        lap_errors.popleft()

                    trail.append(centre_px)

                    if writer is not None:

                        writer.writerow([
                            round(now - started, 3),
                            trajectory.shape,
                            phase,
                            int(tracker.feedforward),
                            round(result["ref"][0], 5),
                            round(result["ref"][1], 5),
                            round(x_m, 5),
                            round(y_m, 5),
                            round(result["error_m"] * 1000, 2),
                            round(roll, 4),
                            round(pitch, 4),
                            round(result["ff_tilt"], 4),
                            round(dt, 4),
                        ])

            # ----- Arduino replies -----

            if ser is not None:

                for reply in read_arduino_replies(ser, rx_buffer):

                    if reply.startswith("ERR:"):

                        print("[ARDUINO ERROR]", reply)
                        print("[RECENT TX]", list(recent_commands))

                        stop("ARDUINO ERROR: PRESS R, THEN SPACE")

            # ----- Display -----

            display, _, _, _ = detector.draw_detection(frame, detection)

            cv2.polylines(display, [outline], False, (255, 200, 0), 2, cv2.LINE_AA)

            if len(trail) > 1:

                cv2.polylines(
                    display,
                    [np.array(trail, dtype=np.int32).reshape(-1, 1, 2)],
                    False,
                    (0, 170, 0),
                    2,
                    cv2.LINE_AA,
                )

            if active and result is not None:

                target = to_pixels(*result["ref"])

                cv2.circle(display, target, 9, (255, 0, 255), 2)
                cv2.circle(display, target, 3, (255, 0, 255), -1)

                if centre_px is not None:
                    cv2.line(display, centre_px, target, (255, 0, 255), 1)

            lines = [f"{mode_name}: {trajectory.shape}, "
                     f"{trajectory.period:g} s per lap"]

            if active and result is not None:

                lines.append(
                    f"{plan.phase(now)}  lap {plan.laps(now):.1f}"
                )

                rms = (
                    math.sqrt(sum(e for _, e in lap_errors) / len(lap_errors))
                    if lap_errors
                    else 0.0
                )

                lines.append(
                    f"Error now {result['error_m'] * 1000:4.1f} mm   "
                    f"RMS last lap {rms * 1000:4.1f} mm"
                )

                lines.append(
                    f"Roll={roll:+.2f} Pitch={pitch:+.2f} deg   "
                    f"feedforward {result['ff_tilt']:.2f} deg"
                )

            else:

                lines.append(status)
                lines.append("" if found else "BALL NOT DETECTED")
                lines.append("")

            lines.append(
                f"Feedforward {'ON' if tracker.feedforward else 'OFF'} (F)"
                + (f"   Loop {1.0 / loop_dt:4.1f} Hz" if loop_dt > 0 else "")
            )

            lines.append("SPACE start/pause  1-5 shape  R neutral  Q quit")

            overlay_text(display, lines)

            cv2.imshow(WINDOW, display)

            key = cv2.waitKey(30 if simulated is not None else 1) & 0xFF

            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

            if key in (ord("q"), 27):
                break

            # ----- Keys -----

            if key == ord(" "):

                if active:
                    stop("PAUSED (neutral)")

                elif found:

                    tracker.reset()
                    trail.clear()

                    plan = Plan(trajectory, (x_m, y_m), time.monotonic())
                    active = True
                    previous = time.monotonic()

                    print(f"Tracking {trajectory.shape}.")

                else:
                    print("Cannot start: ball not detected.")

            elif key == ord("r"):

                stop("PAUSED (neutral)")

                if ser is not None:
                    ser.reset_input_buffer()
                    rx_buffer.clear()

            elif key == ord("f"):

                tracker.feedforward = not tracker.feedforward

                print("Feedforward", "on" if tracker.feedforward else "off")

            elif ord("1") <= key <= ord("0") + len(SHAPES):

                candidate = Trajectory(
                    SHAPES[key - ord("1")], args.size, args.period
                )

                ok, message = check_feasible(candidate, args)
                print(message)

                if ok:

                    trajectory = candidate
                    outline = outline_pixels(trajectory)

                    trail.clear()
                    lap_errors.clear()

                    # Glide from wherever the reference is now to the new shape
                    if active and found:

                        plan = Plan(
                            trajectory,
                            result["ref"] if result else (x_m, y_m),
                            time.monotonic(),
                        )

            # ----- Next frame -----

            if cap is not None:

                ok, frame = cap.read()

                if not ok:
                    print("Camera read failed.")
                    break

    except KeyboardInterrupt:

        print("Interrupted.")

    except Exception as exc:

        print("[DEMO STOPPED]", exc)
        print("Use physical power cutoff if necessary.")

    finally:

        if ser is not None:

            try:
                send_command(ser, neutral, recent_commands)
                time.sleep(0.1)

            except Exception as exc:
                print("Failed to send neutral:", exc)

            ser.close()

        if cap is not None:
            cap.release()

        cv2.destroyAllWindows()

        if csv_file is not None:
            csv_file.close()

        for index, label in ((1, "on"), (0, "off")):

            if error_count[index] > 60:

                rms = math.sqrt(error_sum[index] / error_count[index])

                print(
                    f"RMS tracking error with feedforward {label}: "
                    f"{rms * 1000:.1f} mm"
                )

        print("Demo stopped.")


if __name__ == "__main__":
    main()
