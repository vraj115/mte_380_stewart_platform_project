"""Waypoint trajectory planner for the 3-servo Stewart platform.

Moves the ball between waypoints (a diamond by default) using the model in
the team's write-up, and compares it with plain position (PID) control.

The model
    Ball:            x'' = (3/5) g theta        -> 0.10273 m/s^2 per degree
    Stopping distance: d_stop = v^2 / (2 a_brake)
    Tilt for a wanted acceleration:
                     pitch = a_x / 0.10273,  roll = -a_y / 0.10273

How the planner uses it
    Every move starts from rest and has three phases:
      ACCELERATE  a timed tilt toward the waypoint, sized from the model
      COAST       plate level, ball rolls at the cruise speed
      BRAKE       starts when d_stop reaches the distance left; the tilt is
                  then set from the measured speed and distance,
                  a = v^2 / (2 d), so the ball stops at the waypoint
    Speed and distance are projected one dead time ahead, using the
    commands already sent, before the rule is applied.

Uneven ball
    The logs show a stationary ball does not respond to tilts under about
    2 degrees, but rolls normally once it is moving. So the planner:
      * accelerates with a firm tilt (--go-tilt, 3 deg) that is well above
        that threshold, and raises it if the ball still did not move,
      * plans its braking with a tilt below the threshold, so the ball
        stops and stays instead of being thrown back,
      * never asks for a small tilt to move a resting ball: if the ball
        ends up outside the tolerance circle it plans another short move,
      * PARKED: levels the plate once the ball is slow and inside the
        circle, instead of chasing the last few millimetres,
      * compares the speed each push produced with the plan and corrects
        the model, and learns the plate's level error while the ball rolls.

Usage:
    python waypoint_demo.py --sim                 # simulated uneven ball
    python waypoint_demo.py --report              # figures for the write-up
    python waypoint_demo.py                       # camera only, no servos
    python waypoint_demo.py --live --log          # real platform
    python waypoint_demo.py --live --shape star --trim-roll 0.6 --trim-pitch -0.4

Controls:
    SPACE = Start / pause
    P     = Planner on / off (off = plain PID stepping between waypoints)
    1-4   = diamond / square / triangle / star
    R     = Return to neutral
    Q/ESC = Quit (prints overshoot per control strategy)
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
    limit_vector,
    overlay_text,
    rate_limit_tilt,
    reachable_tilt,
    read_arduino_replies,
    send_command,
)
from trajectory_demo import connect_arduino, open_camera, polygon_vertices


ROOT = Path(__file__).resolve().parent

WINDOW = "Stewart Platform Waypoint Planner"

SHAPES = ["diamond", "square", "triangle", "star"]

# (3/5) * 9.81 m/s^2 per radian, expressed per degree
IDEAL_GAIN = 0.6 * 9.81 * math.pi / 180.0

# Ball counts as stationary below STILL and as rolling above ROLLING (m/s)
STILL_SPEED = 0.012
ROLLING_SPEED = 0.03


def waypoints_for(shape, size):
    """Corner list for one lap, ending back at the centre."""

    if shape == "diamond":
        corners = polygon_vertices(4, 1, size, 0.0)

    elif shape == "square":
        corners = polygon_vertices(4, 1, size, 45.0)

    elif shape == "triangle":
        corners = polygon_vertices(3, 1, size, 90.0)

    elif shape == "star":
        corners = polygon_vertices(5, 2, size, 90.0)

    else:
        raise ValueError(f"Unknown shape: {shape}")

    return corners + [corners[0], (0.0, 0.0)]


# ==================================================
# PLANNER + CONTROLLER
# ==================================================

class WaypointController:
    """Drives the ball through a list of waypoints.

    planner on : brake-distance speed planning (the write-up's model)
    planner off: the balancing PID with its setpoint stepped to each waypoint
    """

    def __init__(self, args, solver, waypoints):

        self.args = args
        self.solver = solver
        self.waypoints = waypoints

        self.planner = not args.no_planner

        self.pid_x = PID(args.kp, args.ki, args.kd, args.d_filter, args.i_limit)
        self.pid_y = PID(args.kp, args.ki, args.kd, args.d_filter, args.i_limit)

        # Level error learned while the ball rolls, in degrees of push
        self.level_x = 0.0
        self.level_y = 0.0

        # Learned from each move: the tilt that frees the ball, and how
        # much speed a push really gives compared with the model
        self.go_tilt = args.go_tilt
        self.model_scale = 1.0

        # One row per finished leg: (planner, overshoot_m, seconds, timed_out)
        self.results = []

        self.reset()

    # ----- state -----

    def reset(self):

        self.pid_x.reset()
        self.pid_y.reset()

        self.index = 0
        self.leg_start = None
        self.leg_time = 0.0
        self.dwell = 0.0
        self.overshoot = 0.0

        self.last_xy = None
        self.vx = 0.0
        self.vy = 0.0
        self.ax = 0.0
        self.ay = 0.0

        self.still_time = self.args.stuck_wait
        self.move = None

        self.history = deque()

        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.saturated = False

    @property
    def target(self):

        return self.waypoints[self.index]

    def next_leg(self, timed_out):

        self.results.append(
            (self.planner, self.overshoot, self.leg_time, timed_out)
        )

        self.leg_start = self.target
        self.index = (self.index + 1) % len(self.waypoints)

        self.leg_time = 0.0
        self.dwell = 0.0
        self.overshoot = 0.0
        self.move = None

        # A ball resting at the last waypoint can be sent off at once
        self.still_time = 0.0 if timed_out else self.args.stuck_wait

        self.pid_x.reset()
        self.pid_y.reset()

    # ----- estimation -----

    def estimate_motion(self, x, y, dt):
        """Filtered velocity and acceleration from camera positions."""

        if self.last_xy is None:
            self.last_xy = (x, y)
            return

        alpha = dt / (0.06 + dt)

        old_vx, old_vy = self.vx, self.vy

        self.vx += alpha * ((x - self.last_xy[0]) / dt - self.vx)
        self.vy += alpha * ((y - self.last_xy[1]) / dt - self.vy)

        beta = dt / (0.15 + dt)

        self.ax += beta * ((self.vx - old_vx) / dt - self.ax)
        self.ay += beta * ((self.vy - old_vy) / dt - self.ay)

        self.last_xy = (x, y)

    def delayed_push(self, t):
        """The push (deg) that is reaching the ball now, one dead time
        after it was commanded."""

        while len(self.history) > 1 and self.history[1][0] <= t - self.args.delay:
            self.history.popleft()

        if self.history and self.history[0][0] <= t - self.args.delay:
            return self.history[0][1], self.history[0][2]

        return 0.0, 0.0

    def predict(self, t, x, y):
        """Where the ball will be, and how fast, when a command sent now
        reaches it: roll the measured state forward through the commands
        that are already on their way."""

        gain = self.args.plant_gain * self.model_scale

        vx, vy = self.vx, self.vy
        previous_time = t - self.args.delay

        # Push acting on the ball during the stretch being stepped over
        acting_x, acting_y = 0.0, 0.0

        for stamp, push_x, push_y in list(self.history) + [(t, 0.0, 0.0)]:

            if stamp > previous_time:

                step = stamp - previous_time
                previous_time = stamp

                x += vx * step
                y += vy * step
                vx += gain * acting_x * step
                vy += gain * acting_y * step

            acting_x, acting_y = push_x, push_y

        return x, y, vx, vy

    # ----- the write-up's model -----

    def plan_move(self, t, x, y):
        """Plan a move from rest at (x, y) to the waypoint: accelerate,
        coast, brake."""

        args = self.args

        target_x, target_y = self.target
        distance = math.hypot(target_x - x, target_y - y)

        # Accelerate hard enough to free the ball; plan to brake gently
        accel = args.plant_gain * self.go_tilt * self.model_scale
        brake = args.brake_accel

        # Highest speed that still leaves room to stop:
        # distance = v^2 / (2 accel) + v^2 / (2 brake)
        peak = min(
            args.cruise_speed,
            math.sqrt(2.0 * distance * accel * brake / (accel + brake)),
        )

        self.move = {
            "start": t,
            "from": (x, y),
            "aim": ((target_x - x) / distance, (target_y - y) / distance),
            "distance": distance,
            "peak": peak,
            "accel_s": peak / accel,
            "phase": "ACCELERATE",
            "top_speed": 0.0,
        }

    def planned_push(self, t, x, y, dt):
        """Wanted push in degrees (x, y) and a label for the display."""

        args = self.args
        gain = args.plant_gain * self.model_scale

        target_x, target_y = self.target

        speed = math.hypot(self.vx, self.vy)
        distance = math.hypot(target_x - x, target_y - y)

        if speed < STILL_SPEED:
            self.still_time += dt

        elif speed > ROLLING_SPEED:
            self.still_time = 0.0

        # ----- A planned move is running -----

        if self.move is not None:

            move = self.move
            elapsed = t - move["start"]
            aim_x, aim_y = move["aim"]

            # State at the moment a command sent now takes effect
            px, py, pvx, pvy = self.predict(t, x, y)

            remaining = (target_x - px) * aim_x + (target_y - py) * aim_y
            v_along = pvx * aim_x + pvy * aim_y

            # Sideways: gentle feedback back onto the line
            offset = (px - target_x) * -aim_y + (py - target_y) * aim_x
            v_across = pvx * -aim_y + pvy * aim_x

            side = -(args.kp * offset + args.kd * v_across)
            side_x, side_y = side * -aim_y, side * aim_x

            measured_along = self.vx * aim_x + self.vy * aim_y
            move["top_speed"] = max(move["top_speed"], measured_along)

            if move["phase"] == "ACCELERATE":

                if elapsed < move["accel_s"]:

                    return (
                        self.go_tilt * aim_x, self.go_tilt * aim_y,
                        "ACCELERATE",
                    )

                move["phase"] = "COAST"

            # Uneven ball: did the push free it? If not, push harder.
            if (
                move["top_speed"] < ROLLING_SPEED
                and elapsed > move["accel_s"] + args.delay + 0.3
            ):

                self.go_tilt = min(0.9 * args.max_tilt, self.go_tilt + 0.5)
                self.move = None
                self.still_time = args.stuck_wait

                return 0.0, 0.0, "STUCK: RETRY"

            # It moved but has come to rest again part-way: plan afresh
            if (
                elapsed > move["accel_s"] + args.delay + 0.3
                and self.still_time > args.stuck_wait
            ):

                self.move = None

                return 0.0, 0.0, "COAST"

            # The push has fully arrived: compare the speed reached with
            # the plan and correct the model for the next move.
            if "learned" not in move and elapsed > move["accel_s"] + args.delay + 0.1:

                move["learned"] = True

                if move["top_speed"] > ROLLING_SPEED:

                    ratio = max(0.6, min(1.6, move["top_speed"] / move["peak"]))

                    # Kept within a narrow band: one odd push (ball held
                    # back by a slope, or already rolling) must not
                    # stretch the next pushes into a launch.
                    self.model_scale = max(
                        0.8, min(1.3, self.model_scale * math.sqrt(ratio))
                    )

            if move["phase"] == "COAST":

                # The write-up's rule: start braking when the stopping
                # distance v^2 / (2 a_brake) reaches the distance left.
                stop_distance = v_along * v_along / (2.0 * args.brake_accel)

                if v_along > 0 and stop_distance >= remaining:
                    move["phase"] = "BRAKE"

                elif elapsed > move["accel_s"] + args.delay and v_along < 0.01:
                    # Stopped short on its own
                    self.move = None
                    self.still_time = 0.0
                    return side_x, side_y, "COAST"

                else:

                    # Plate level; a light touch keeps the planned speed
                    hold = max(-1.0, min(1.0, 1.5 * (move["peak"] - v_along) / gain))

                    if elapsed < move["accel_s"] + args.delay:
                        hold = 0.0

                    return (
                        hold * aim_x + side_x, hold * aim_y + side_y, "COAST",
                    )

            # BRAKE: the deceleration that stops the ball exactly at the
            # waypoint from where it is now, a = v^2 / (2 d)
            if v_along < 0.012:

                self.move = None
                self.still_time = 0.0

                return 0.0, 0.0, "BRAKE"

            limit = 0.8 * args.max_tilt * gain

            if remaining > 0.002:
                decel = min(limit, v_along * v_along / (2.0 * remaining))
            else:
                decel = limit

            return (
                -decel / gain * aim_x + side_x,
                -decel / gain * aim_y + side_y,
                "BRAKE",
            )

        # ----- No move running -----

        if distance < args.tolerance and speed < STILL_SPEED:

            # Close enough. Only damp what little motion is left and let
            # the ball rest; pushing it toward the exact point only makes
            # it hop past the waypoint.
            return -args.kd * self.vx, -args.kd * self.vy, "PARKED"

        if self.still_time > args.stuck_wait:

            # At rest away from the waypoint: plan a move from here
            self.plan_move(t, x, y)

            aim_x, aim_y = self.move["aim"]

            return self.go_tilt * aim_x, self.go_tilt * aim_y, "ACCELERATE"

        # Rolling toward the waypoint. Plain position feedback lets the
        # ball build up speed over a long distance and then cannot stop
        # it. Instead, choose a speed that is safe for the distance left
        # and steer the ball's velocity to it.
        px, py, pvx, pvy = self.predict(t, x, y)

        left_x, left_y = target_x - px, target_y - py
        left = max(1e-6, math.hypot(left_x, left_y))

        # Never faster than the ball can stop from (v = sqrt(2 a d)),
        # never faster than the cruise speed, and slowing in proportion
        # to the distance on the final approach.
        wanted = min(
            args.cruise_speed,
            args.approach_gain * left,
            math.sqrt(2.0 * args.brake_accel * left),
        )

        push_x = args.speed_gain * (wanted * left_x / left - pvx) / gain
        push_y = args.speed_gain * (wanted * left_y / left - pvy) / gain

        return push_x, push_y, "APPROACH"

    def pid_push(self, x, y, dt):

        target_x, target_y = self.target

        integrate = not self.saturated

        return (
            self.pid_x.update(target_x - x, dt, integrate),
            self.pid_y.update(target_y - y, dt, integrate),
            "PID",
        )

    # ----- one control step -----

    def update(self, t, x, y, dt):

        args = self.args

        dt = max(0.001, min(dt, 0.2))

        if self.leg_start is None:
            self.leg_start = (x, y)

        self.estimate_motion(x, y, dt)

        speed = math.hypot(self.vx, self.vy)

        # Learn the level error: while rolling, any acceleration the
        # commanded tilt does not explain is the plate being off level.
        delayed_x, delayed_y = self.delayed_push(t)

        if args.learn_level and speed > ROLLING_SPEED:

            rate = 0.4 * dt
            limit = 1.5

            self.level_x += rate * (self.ax / args.plant_gain - delayed_x)
            self.level_y += rate * (self.ay / args.plant_gain - delayed_y)

            self.level_x = max(-limit, min(limit, self.level_x))
            self.level_y = max(-limit, min(limit, self.level_y))

        # Leg bookkeeping
        target_x, target_y = self.target
        distance = math.hypot(target_x - x, target_y - y)

        self.leg_time += dt

        leg_x = target_x - self.leg_start[0]
        leg_y = target_y - self.leg_start[1]
        length = math.hypot(leg_x, leg_y)

        if length > 0.01:

            # Overshoot: how far past the waypoint, along the approach
            past = ((x - target_x) * leg_x + (y - target_y) * leg_y) / length
            self.overshoot = max(self.overshoot, past)

        if args.dwell <= 0.0:

            # Touch and go: the waypoint is reached the moment the ball's
            # centre enters the target circle, whatever its speed.
            arrived = distance < args.tolerance

        else:

            if distance < args.tolerance and speed < 0.02:
                self.dwell += dt
            else:
                self.dwell = 0.0

            arrived = self.dwell >= args.dwell

        if arrived:
            self.next_leg(False)

        elif self.leg_time > args.leg_timeout:
            self.next_leg(True)

        # Wanted push
        if self.planner:
            push_x, push_y, label = self.planned_push(t, x, y, dt)
        else:
            push_x, push_y, label = self.pid_push(x, y, dt)

        # Edge guard: near the rim, add a push back toward the centre
        radius = math.hypot(x, y)

        if radius > args.edge_radius:

            inward = args.edge_gain * (radius - args.edge_radius)

            push_x -= inward * x / radius
            push_y -= inward * y / radius

        self.history.append((t, push_x, push_y))

        # +pitch accelerates the ball toward +x, +roll toward -y
        pitch = args.pitch_sign * (push_x - self.level_x) + args.trim_pitch
        roll = args.roll_sign * (push_y - self.level_y) + args.trim_roll

        wanted = math.hypot(roll, pitch)

        roll, pitch = limit_vector(roll, pitch, args.max_tilt)

        roll, pitch = rate_limit_tilt(
            roll, pitch, self.last_roll, self.last_pitch,
            args.tilt_rate * dt,
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
            "label": label,
            "target": self.target,
            "distance": math.hypot(self.target[0] - x, self.target[1] - y),
            "speed": speed,
            "stop_distance": speed * speed / (2.0 * args.brake_accel),
        }


def summarise(results):
    """Lines of text comparing the two strategies."""

    lines = []

    for planner, name in ((True, "Trajectory planner"), (False, "Position control (PID)")):

        legs = [r for r in results if r[0] == planner]

        if not legs:
            continue

        overshoot = [max(0.0, r[1]) * 1000 for r in legs]
        seconds = [r[2] for r in legs]
        timeouts = sum(1 for r in legs if r[3])

        lines.append(
            f"{name}: {len(legs)} legs, overshoot mean "
            f"{sum(overshoot) / len(legs):.1f} mm, worst {max(overshoot):.1f} mm, "
            f"mean time per leg {sum(seconds) / len(legs):.1f} s"
            + (f", {timeouts} timed out" if timeouts else "")
        )

    return lines


# ==================================================
# SIMULATED BALL
# ==================================================

class SimulatedBall:
    """Rolling ball with dead time. With `uneven` set it behaves like the
    logged ball: at rest it stays put until the net tilt passes a
    threshold that changes every time it stops (1.3 to 2.4 deg), it rolls
    with a wobble from its heavy side, and the plate is slightly off level."""

    def __init__(self, uneven, plate_radius, delay=0.25):

        self.uneven = uneven
        self.gain = 0.07 if uneven else IDEAL_GAIN
        self.delay = delay if uneven else 0.0
        self.level = (0.3, 1.0) if uneven else (0.0, 0.0)
        self.stick_range = (0.6, 1.6)
        self.noise = 0.0003 if uneven else 0.0
        self.wobble_deg = 0.5 if uneven else 0.0

        self.plate_radius = plate_radius
        self.radius = 0.02

        self.x, self.y = 0.0, 0.0
        self.vx, self.vy = 0.0, 0.0

        self.random = random.Random(7)
        self.stuck = uneven
        self.threshold = 1.2

        # Direction of the heavy point (unit vector), starting at the bottom
        self.heavy = np.array([0.0, 0.0, -1.0])

        self.history = deque()

    def step(self, t, dt, push_x, push_y):

        self.history.append((t, push_x, push_y))

        while len(self.history) > 1 and self.history[1][0] <= t - self.delay:
            self.history.popleft()

        delayed_x, delayed_y = 0.0, 0.0

        if self.history[0][0] <= t - self.delay:
            _, delayed_x, delayed_y = self.history[0]

        net_x = delayed_x + self.level[0]
        net_y = delayed_y + self.level[1]

        remaining = min(dt, 0.1)

        while remaining > 1e-9:

            h = min(remaining, 0.004)
            remaining -= h

            if self.stuck:

                if math.hypot(net_x, net_y) <= self.threshold:
                    continue

                self.stuck = False

            speed = math.hypot(self.vx, self.vy)

            ax = self.gain * (net_x + self.wobble_deg * self.heavy[0])
            ay = self.gain * (net_y + self.wobble_deg * self.heavy[1])

            if self.uneven and speed > 1e-6:

                # Rolling resistance
                drag = 0.012 + 0.3 * speed

                ax -= drag * self.vx / speed
                ay -= drag * self.vy / speed

            self.vx += ax * h
            self.vy += ay * h

            self.x += self.vx * h
            self.y += self.vy * h

            # Rolling without slipping turns the heavy point
            omega = np.array([-self.vy, self.vx, 0.0]) / self.radius

            self.heavy = self.heavy + np.cross(omega, self.heavy) * h
            self.heavy /= np.linalg.norm(self.heavy)

            if (
                self.uneven
                and math.hypot(self.vx, self.vy) < 0.012
                and math.hypot(net_x, net_y) < 0.8 * self.threshold
            ):

                self.stuck = True
                self.vx = 0.0
                self.vy = 0.0
                self.threshold = self.random.uniform(*self.stick_range)

        distance = math.hypot(self.x, self.y)
        rim = self.plate_radius - self.radius

        if distance > rim:

            self.x *= rim / distance
            self.y *= rim / distance
            self.vx = 0.0
            self.vy = 0.0

    def measure(self):

        return (
            self.x + self.random.gauss(0.0, self.noise),
            self.y + self.random.gauss(0.0, self.noise),
        )


# ==================================================
# REPORT (figures for the write-up)
# ==================================================

def run_headless(args, solver, uneven, planner, laps=1, limit_s=150.0):
    """Simulate without a window. Returns the log and the leg results."""

    waypoints = waypoints_for(args.shape, args.size)

    controller = WaypointController(args, solver, waypoints)
    controller.planner = planner

    ball = SimulatedBall(uneven, 0.15, args.delay)

    dt = 1.0 / 30.0
    t = 0.0
    roll = pitch = 0.0
    log = []

    while len(controller.results) < laps * len(waypoints) and t < limit_s:

        ball.step(t, dt, args.pitch_sign * pitch, args.roll_sign * roll)

        x, y = ball.measure()

        out = controller.update(t, x, y, dt)

        roll, pitch = out["roll"], out["pitch"]

        log.append((
            t, ball.x, ball.y, math.hypot(ball.vx, ball.vy),
            roll, pitch, out["target"][0], out["target"][1],
        ))

        t += dt

    return np.array(log), controller.results


def make_report(args, solver):

    import matplotlib

    matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    folder = ROOT / "waypoint_report"
    folder.mkdir(exist_ok=True)

    # Overshoot past a waypoint can only be measured if the ball has to
    # come to rest there, so the report does not use touch-and-go.
    if args.dwell <= 0.0:
        args.dwell = 0.25
        args.tolerance = max(args.tolerance, 0.02)

    waypoints = waypoints_for(args.shape, args.size)
    outline = np.array([(0.0, 0.0)] + waypoints) * 1000

    colours = {True: "#0b6e99", False: "#d1495b"}
    names = {True: "Trajectory planner", False: "Position control (PID)"}

    for uneven, tag, title in (
        (False, "ideal", "ideal ball (the write-up's model)"),
        (True, "uneven", "sticking ball, 0.25 s dead time, plate off level"),
    ):

        runs = {
            planner: run_headless(args, solver, uneven, planner)
            for planner in (True, False)
        }

        print(f"\n{args.shape}, {title}:")

        for line in summarise(runs[True][1] + runs[False][1]):
            print("  " + line)

        # Figure 1: X-Y path
        fig, ax = plt.subplots(figsize=(6.4, 6.0))

        ax.plot(outline[:, 0], outline[:, 1], "--", color="#555555",
                label="Desired path", zorder=1)
        ax.plot(outline[1:, 0], outline[1:, 1], "o", color="#222222",
                markersize=7, zorder=3)

        for planner in (False, True):
            log = runs[planner][0]
            ax.plot(log[:, 1] * 1000, log[:, 2] * 1000,
                    color=colours[planner], label=names[planner], linewidth=1.6)

        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title(f"Ball path: {title}")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

        fig.tight_layout()
        fig.savefig(folder / f"fig1_xy_path_{tag}.png", dpi=150)
        plt.close(fig)

        # Figure 2: position vs time
        fig, axes = plt.subplots(2, 1, figsize=(8.0, 5.6), sharex=True)

        for row, (column, target, name) in enumerate(((1, 6, "x"), (2, 7, "y"))):

            for planner in (False, True):

                log = runs[planner][0]

                axes[row].plot(log[:, 0], log[:, column] * 1000,
                               color=colours[planner], label=names[planner])
                axes[row].plot(log[:, 0], log[:, target] * 1000, ":",
                               color=colours[planner], linewidth=1.0)

            axes[row].set_ylabel(f"{name} (mm)")
            axes[row].grid(alpha=0.3)

        axes[0].set_title(f"Ball position (dotted = waypoint): {title}")
        axes[0].legend(fontsize=8)
        axes[1].set_xlabel("Time (s)")

        fig.tight_layout()
        fig.savefig(folder / f"fig2_position_{tag}.png", dpi=150)
        plt.close(fig)

        # Figure 3: speed vs time
        fig, ax = plt.subplots(figsize=(8.0, 3.6))

        for planner in (False, True):
            log = runs[planner][0]
            ax.plot(log[:, 0], log[:, 3] * 1000,
                    color=colours[planner], label=names[planner])

        ax.axhline(args.cruise_speed * 1000, color="#555555", linestyle="--",
                   linewidth=1.0, label="Planner cruise speed")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Ball speed (mm/s)")
        ax.set_title(f"Ball speed: {title}")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

        fig.tight_layout()
        fig.savefig(folder / f"fig3_speed_{tag}.png", dpi=150)
        plt.close(fig)

        # Figure 4: roll and pitch vs time
        fig, axes = plt.subplots(2, 1, figsize=(8.0, 5.6), sharex=True)

        for row, (column, name) in enumerate(((4, "Roll"), (5, "Pitch"))):

            for planner in (False, True):
                log = runs[planner][0]
                axes[row].plot(log[:, 0], log[:, column],
                               color=colours[planner], label=names[planner])

            for limit in (-args.max_tilt, args.max_tilt):
                axes[row].axhline(limit, color="#555555", linestyle="--",
                                  linewidth=1.0)

            axes[row].set_ylabel(f"{name} (deg)")
            axes[row].grid(alpha=0.3)

        axes[0].set_title(f"Commanded tilt (dashed = limit): {title}")
        axes[0].legend(fontsize=8)
        axes[1].set_xlabel("Time (s)")

        fig.tight_layout()
        fig.savefig(folder / f"fig4_tilt_{tag}.png", dpi=150)
        plt.close(fig)

    print(f"\nFigures saved in {folder}")


# ==================================================
# COMMAND LINE
# ==================================================

def parse_args(argv=None):

    parser = argparse.ArgumentParser(
        description="Waypoint trajectory planner (stopping-distance model)"
    )

    parser.add_argument("--live", action="store_true",
                        help="send servo commands to the Arduino")
    parser.add_argument("--sim", action="store_true",
                        help="simulated uneven ball; no hardware needed")
    parser.add_argument("--report", action="store_true",
                        help="simulate both strategies and save figures")
    parser.add_argument("--port", default=None)

    parser.add_argument("--shape", choices=SHAPES, default="diamond")
    parser.add_argument("--size", type=float, default=0.08,
                        help="centre-to-corner distance in m (default 0.08)")

    parser.add_argument("--no-planner", action="store_true",
                        help="start with plain PID position control (P toggles)")

    # The write-up's model
    parser.add_argument("--plant-gain", type=float, default=0.075,
                        help="ball acceleration in m/s^2 per deg of tilt. "
                             "The ideal value is 0.10273 (3/5 g); the logs "
                             "from this platform give about 0.075 (default)")
    parser.add_argument("--brake-accel", type=float, default=0.12,
                        help="planned braking deceleration in m/s^2; keep it "
                             "under plant gain x sticking tilt (default 0.12)")
    parser.add_argument("--cruise-speed", type=float, default=0.07,
                        help="top speed between waypoints in m/s (default 0.07)")
    parser.add_argument("--delay", type=float, default=0.30,
                        help="loop dead time in s (default 0.30)")
    parser.add_argument("--speed-gain", type=float, default=1.3,
                        help="how hard a rolling ball is steered to the wanted "
                             "speed, 1/s; lower = calmer (default 1.3)")
    parser.add_argument("--approach-gain", type=float, default=0.8,
                        help="wanted speed per metre left on the final "
                             "approach, 1/s; lower = slower arrival (default 0.8)")
    parser.add_argument("--edge-radius", type=float, default=0.10,
                        help="beyond this distance from the centre the ball is "
                             "pushed back, m (default 0.10)")
    parser.add_argument("--edge-gain", type=float, default=60.0,
                        help="edge push in deg per m past the edge radius "
                             "(default 60)")

    # Uneven ball
    parser.add_argument("--tolerance", type=float, default=0.013,
                        help="radius of the target circle in m (default 0.013)")
    parser.add_argument("--go-tilt", type=float, default=3.0,
                        help="tilt used to start a stationary ball, deg; must "
                             "be above the tilt it sticks at (default 3)")
    parser.add_argument("--stuck-wait", type=float, default=0.35,
                        help="seconds stationary before the ball counts as "
                             "stuck (default 0.35)")
    parser.add_argument("--no-learn-level", dest="learn_level",
                        action="store_false",
                        help="do not estimate the plate's level error")

    parser.add_argument("--dwell", type=float, default=0.0,
                        help="seconds the ball must rest inside the circle; "
                             "0 = move on the moment its centre enters "
                             "(default 0)")
    parser.add_argument("--leg-timeout", type=float, default=12.0)

    # Same PID and limits as pid_controller.py (used when the planner is off)
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
                        help="write waypoint_run_<time>.csv")

    args = parser.parse_args(argv)

    if sum((args.live, args.sim, args.report)) > 1:
        parser.error("Use only one of --live, --sim, --report")

    checks = (
        (0.02 <= args.size <= 0.10, "Size must be between 0.02 and 0.10 m"),
        (0.02 <= args.plant_gain <= 0.3, "Plant gain must be 0.02 to 0.3"),
        (0.02 <= args.brake_accel <= 0.4, "Brake accel must be 0.02 to 0.4 m/s^2"),
        (0.02 <= args.cruise_speed <= 0.3, "Cruise speed must be 0.02 to 0.3 m/s"),
        (0.0 <= args.delay <= 1.0, "Delay must be between 0 and 1 s"),
        (0.3 <= args.speed_gain <= 5.0, "Speed gain must be 0.3 to 5"),
        (0.2 <= args.approach_gain <= 4.0, "Approach gain must be 0.2 to 4"),
        (0.05 <= args.edge_radius <= 0.14, "Edge radius must be 0.05 to 0.14 m"),
        (0.0 <= args.edge_gain <= 200.0, "Edge gain must be 0 to 200"),
        (0.005 <= args.tolerance <= 0.05, "Tolerance must be 0.005 to 0.05 m"),
        (0.5 <= args.go_tilt <= 0.9 * args.max_tilt,
         "Go tilt must be between 0.5 deg and 90% of the maximum tilt"),
        (0 < args.max_tilt <= 15.0, "Maximum tilt must be between 0 and 15 deg"),
        (0 <= args.i_limit <= args.max_tilt, "Integral limit out of range"),
    )

    for ok, message in checks:
        if not ok:
            parser.error(message)

    return args


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

    if args.report:
        make_report(args, solver)
        return

    with open(ROOT / "stewart_config.json", encoding="utf-8") as file:
        config = json.load(file)

    port = args.port or config["servos"]["port"]
    baud = int(config["servos"].get("baud_rate", 9600))

    neutral = solver.format_serial_command(
        solver.calculate_servo_angles(0.0, 0.0), SERIAL_DECIMALS
    )

    controller = WaypointController(
        args, solver, waypoints_for(args.shape, args.size)
    )

    shape = args.shape
    scale = detector.pixel_to_meter_ratio
    centre_x, centre_y = detector.platform_center

    def to_pixels(x_m, y_m):

        return (
            int(round(centre_x + x_m / scale)),
            int(round(centre_y - y_m / scale)),
        )

    ser = None
    cap = None
    simulated = None
    csv_file = None
    writer = None

    recent_commands = deque(maxlen=10)
    rx_buffer = bytearray()
    last_command = neutral

    active = False
    status = "PRESS SPACE TO START"
    result = None
    roll = 0.0
    pitch = 0.0
    loop_dt = 0.0

    trail = deque(maxlen=450)

    def stop(reason):

        nonlocal active, status, last_command, roll, pitch, result

        active = False
        status = reason
        result = None
        roll = 0.0
        pitch = 0.0

        controller.reset()

        if ser is not None:
            send_command(ser, neutral, recent_commands)
            last_command = neutral

    try:

        if args.sim:

            simulated = SimulatedBall(
                True, detector.platform_radius_px * scale, args.delay
            )

            simulated.x, simulated.y = 0.03, -0.02

            blank = np.full(
                (detector.frame_height, detector.frame_width, 3),
                (60, 60, 60), dtype=np.uint8,
            )

            cv2.circle(
                blank, detector.platform_center,
                detector.platform_radius_px, (205, 205, 205), -1,
            )

        else:

            cap = open_camera(detector)

            ok, frame = cap.read()

            if not ok:
                raise RuntimeError("Camera did not return a frame")

        mode_name = (
            "SIMULATION" if args.sim else "LIVE" if args.live else "CAMERA ONLY"
        )

        print("\nMode:", mode_name)
        print(
            f"Model: {args.plant_gain:.5f} m/s^2 per deg, brake "
            f"{args.brake_accel} m/s^2, cruise {args.cruise_speed} m/s, "
            f"dead time {args.delay} s"
        )
        print("SPACE: start/pause | P: planner on/off | 1-4: shape | "
              "R: neutral | Q: quit")

        if args.log:

            log_path = ROOT / f"waypoint_run_{datetime.now():%Y%m%d_%H%M%S}.csv"

            csv_file = open(log_path, "w", newline="", encoding="utf-8")
            writer = csv.writer(csv_file)

            writer.writerow([
                "elapsed_s", "planner", "phase", "target_x_m", "target_y_m",
                "ball_x_m", "ball_y_m", "speed_m_s", "distance_mm",
                "roll_deg", "pitch_deg", "level_x_deg", "level_y_deg", "dt_s",
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
                        now, dt,
                        args.pitch_sign * pitch, args.roll_sign * roll,
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
                    result = controller.update(now, x_m, y_m, dt)

                except (KinematicsError, ValueError) as exc:
                    stop(f"IK REJECTED: {exc}")
                    print(status)

                else:

                    roll = result["roll"]
                    pitch = result["pitch"]

                    if ser is not None and result["command"] != last_command:

                        send_command(ser, result["command"], recent_commands)
                        last_command = result["command"]

                    trail.append(centre_px)

                    if writer is not None:

                        writer.writerow([
                            round(now - started, 3),
                            int(controller.planner),
                            result["label"],
                            round(result["target"][0], 5),
                            round(result["target"][1], 5),
                            round(x_m, 5),
                            round(y_m, 5),
                            round(result["speed"], 4),
                            round(result["distance"] * 1000, 2),
                            round(roll, 4),
                            round(pitch, 4),
                            round(controller.level_x, 3),
                            round(controller.level_y, 3),
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

            corners = [to_pixels(0.0, 0.0)] + [
                to_pixels(*p) for p in controller.waypoints
            ]

            cv2.polylines(
                display,
                [np.array(corners, dtype=np.int32).reshape(-1, 1, 2)],
                False, (255, 200, 0), 2, cv2.LINE_AA,
            )

            tolerance_px = max(2, int(round(args.tolerance / scale)))

            for corner in corners:
                cv2.circle(display, corner, tolerance_px, (255, 200, 0), 1,
                           cv2.LINE_AA)

            if len(trail) > 1:

                cv2.polylines(
                    display,
                    [np.array(trail, dtype=np.int32).reshape(-1, 1, 2)],
                    False, (0, 170, 0), 2, cv2.LINE_AA,
                )

            strategy = "PLANNER" if controller.planner else "PID ONLY"

            lines = [f"{mode_name}: {shape}   {strategy} (P)"]

            if active and result is not None:

                target_px = to_pixels(*result["target"])

                cv2.circle(display, target_px, tolerance_px, (255, 0, 255), 2,
                           cv2.LINE_AA)
                cv2.circle(display, target_px, 3, (255, 0, 255), -1)

                lines.append(
                    f"{result['label']}   waypoint "
                    f"{controller.index + 1}/{len(controller.waypoints)}"
                )

                lines.append(
                    f"Distance {result['distance'] * 1000:5.1f} mm   "
                    f"stop distance {result['stop_distance'] * 1000:5.1f} mm"
                )

                lines.append(
                    f"Speed {result['speed'] * 1000:5.1f} mm/s   "
                    f"Roll={roll:+.2f} Pitch={pitch:+.2f} deg"
                )

            else:

                lines.append(status)
                lines.append("" if found else "BALL NOT DETECTED")
                lines.append("")

            lines.append(
                f"Level est x={controller.level_x:+.2f} "
                f"y={controller.level_y:+.2f} deg"
                + (f"   Loop {1.0 / loop_dt:4.1f} Hz" if loop_dt > 0 else "")
            )

            lines.append("SPACE start/pause  1-4 shape  R neutral  Q quit")

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

                    controller.reset()
                    trail.clear()

                    active = True
                    previous = time.monotonic()

                    print(f"Running {shape} with the "
                          f"{'planner' if controller.planner else 'PID only'}.")

                else:
                    print("Cannot start: ball not detected.")

            elif key == ord("r"):

                stop("PAUSED (neutral)")

                if ser is not None:
                    ser.reset_input_buffer()
                    rx_buffer.clear()

            elif key == ord("p"):

                controller.planner = not controller.planner

                controller.pid_x.reset()
                controller.pid_y.reset()
                controller.move = None
                controller.still_time = 0.0

                print("Planner", "on" if controller.planner else
                      "off (PID position control)")

            elif ord("1") <= key <= ord("0") + len(SHAPES):

                shape = SHAPES[key - ord("1")]

                controller.waypoints = waypoints_for(shape, args.size)
                controller.index = 0
                controller.leg_start = None
                controller.leg_time = 0.0
                controller.overshoot = 0.0

                trail.clear()

                print("Shape:", shape)

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

        for line in summarise(controller.results):
            print(line)

        if abs(controller.level_x) > 0.2 or abs(controller.level_y) > 0.2:

            print(
                "Level error learned this run. To start with it next time add "
                f"--trim-pitch {args.trim_pitch - args.pitch_sign * controller.level_x:.2f} "
                f"--trim-roll {args.trim_roll - args.roll_sign * controller.level_y:.2f}"
            )

        print("Demo stopped.")


if __name__ == "__main__":
    main()
