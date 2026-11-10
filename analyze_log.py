"""Measure how the platform actually behaves from pid_run_*.csv logs.

    python analyze_log.py                  # every pid_run_*.csv in this folder
    python analyze_log.py pid_run_X.csv    # specific files

For the ACTIVE (balancing) parts of the logs this fits

    ball acceleration(t) = M @ [roll, pitch](t - delay) + bias

and reports:
    * delay  - total dead time from command to ball response
    * gain   - m/s^2 of ball acceleration per commanded degree
    * angle  - rotation between camera axes and platform axes (should be ~0)
    * bias   - acceleration at zero command, i.e. how far off level the
               plate is at neutral, and the trim that would cancel it

Needs only numpy. Uses at least ~30 s of balancing data to be meaningful.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

# Rolling hollow sphere: a = (3/5) g sin(tilt), per degree of tilt
IDEAL_GAIN = 0.6 * 9.81 * math.pi / 180.0
STEP = 0.1  # s, resampling grid


def load_segments(path):
    """Return a list of contiguous balancing segments as numpy arrays."""
    segments = []
    current = []
    with open(path, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if row["mode"] == "ACTIVE" and row["found"] == "1":
                current.append([
                    float(row["elapsed_s"]),
                    float(row["ball_x_m"]),
                    float(row["ball_y_m"]),
                    float(row["roll_deg"]),
                    float(row["pitch_deg"]),
                ])
            else:
                if len(current) >= 20:
                    segments.append(np.array(current))
                current = []
    if len(current) >= 20:
        segments.append(np.array(current))
    return segments


def smooth(values):
    kernel = np.ones(5) / 5.0
    return np.convolve(values, kernel, "same")


def regression_rows(segments, delay):
    inputs, outputs = [], []
    for seg in segments:
        t, x, y, roll, pitch = seg.T
        if t[-1] - t[0] < 3.0:
            continue
        grid = np.arange(t[0], t[-1], STEP)
        xs = smooth(np.interp(grid, t, x))
        ys = smooth(np.interp(grid, t, y))
        ax = np.gradient(np.gradient(xs, STEP), STEP)
        ay = np.gradient(np.gradient(ys, STEP), STEP)
        r = smooth(np.interp(grid - delay, t, roll))
        p = smooth(np.interp(grid - delay, t, pitch))
        # Ignore samples near the rim (wall contact) and segment ends
        keep = np.hypot(xs, ys) < 0.115
        keep[:6] = False
        keep[-6:] = False
        inputs.append(np.c_[r, p, np.ones_like(r)][keep])
        outputs.append(np.c_[ax, ay][keep])
    if not inputs:
        return np.zeros((0, 3)), np.zeros((0, 2))
    return np.vstack(inputs), np.vstack(outputs)


def describe_runs(path, segments):
    print(f"\n{path.name}")
    if not segments:
        print("  no balancing data")
        return
    for seg in segments:
        t, x, y, roll, pitch = seg.T
        radius = np.hypot(x, y) * 1000.0
        dt = np.median(np.diff(t))
        tilt = np.hypot(roll, pitch)
        angle = np.unwrap(np.arctan2(y, x))
        turns = (angle[-1] - angle[0]) / (2 * math.pi)
        print(
            f"  {t[0]:6.1f}-{t[-1]:6.1f} s: {1 / dt:4.1f} Hz, "
            f"radius start {radius[0]:3.0f} / max {radius.max():3.0f} / "
            f"end {radius[-1]:3.0f} mm, rms {np.sqrt((radius ** 2).mean()):3.0f} mm, "
            f"{turns:+.1f} turns around centre, "
            f"tilt at limit {100 * (tilt >= tilt.max() - 0.01).mean():3.0f}% of the time"
        )


def main():
    folder = Path(__file__).resolve().parent
    paths = [Path(p) for p in sys.argv[1:]] or sorted(folder.glob("pid_run_*.csv"))
    if not paths:
        print("No pid_run_*.csv files found. Run pid_controller.py --live --log first.")
        return

    all_segments = []
    for path in paths:
        segments = load_segments(path)
        describe_runs(path, segments)
        all_segments.extend(segments)

    best = None
    for delay in np.arange(0.0, 0.61, 0.02):
        A, Y = regression_rows(all_segments, delay)
        if len(A) < 150:
            print("\nNot enough balancing data away from the rim to fit a model.")
            return
        coef = np.linalg.lstsq(A, Y, rcond=None)[0]
        residual = ((Y - A @ coef) ** 2).sum()
        total = ((Y - Y.mean(0)) ** 2).sum()
        r2 = 1.0 - residual / total
        if best is None or r2 > best[0]:
            best = (r2, delay, coef, len(A))

    r2, delay, coef, count = best
    M = coef[:2].T          # [[ax/roll, ax/pitch], [ay/roll, ay/pitch]]
    bias = coef[2]

    # Ideal mapping is ax = +g*pitch, ay = -g*roll. Express the measured
    # map as (gain, rotation) of that ideal plus a leftover mismatch.
    N = M @ np.array([[0.0, -1.0], [1.0, 0.0]])
    rotation = complex(N[0, 0] + N[1, 1], N[1, 0] - N[0, 1]) / 2
    mismatch = complex(N[0, 0] - N[1, 1], N[1, 0] + N[0, 1]) / 2
    gain = abs(rotation)
    angle = math.degrees(math.atan2(rotation.imag, rotation.real))

    print(f"\nModel fit over {count} samples (R^2 = {r2:.2f})")
    print(f"  dead time           : {delay:.2f} s")
    print(
        f"  gain                : {gain:.3f} m/s^2 per commanded degree "
        f"({gain / IDEAL_GAIN:.0%} of the ideal {IDEAL_GAIN:.3f})"
    )
    print(f"  axis rotation       : {angle:+.1f} deg (0 = camera and platform axes agree)")
    print(f"  x/y mismatch        : {abs(mismatch) / max(gain, 1e-9):.0%} of the gain")
    print(f"  bias at zero command: ax {bias[0]:+.3f}, ay {bias[1]:+.3f} m/s^2")
    if gain > 1e-6:
        trim = -np.linalg.solve(M, bias)
        print(
            f"  trim that cancels it: --trim-roll {trim[0]:+.2f} "
            f"--trim-pitch {trim[1]:+.2f}"
        )

    if r2 < 0.4:
        print("  Low R^2: treat these numbers as rough.")
    if abs(angle) > 15:
        print("  WARNING: axes are rotated. Check --roll-sign/--pitch-sign and servo order.")
    if gain > 1e-6 and delay > 0:
        # PD design for ~45 deg phase margin with this delay and gain
        crossover = 0.55 / (delay + 0.06)
        ratio = math.tan(math.radians(75)) / crossover
        kp = crossover ** 2 / (gain * math.hypot(1.0, crossover * ratio))
        print(
            f"  suggested start     : --kp {kp:.0f} --kd {kp * ratio:.0f} "
            f"--ki {kp * 0.4:.0f}   (reduce the delay to be able to use more gain)"
        )


if __name__ == "__main__":
    main()
