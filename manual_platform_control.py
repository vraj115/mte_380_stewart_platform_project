
"""Small-roll/pitch test of the Stewart platform, with safe dry-run default.

Uses the existing kinematics.py and stewart_config.json files.
    python manual_platform_control.py               # math only
    python manual_platform_control.py --live        # send via COM port after ARM

Windows: press W/S/A/D/R/Q without Enter. Other systems: type key + Enter.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

from kinematics import KinematicsError, StewartKinematics


def read_key() -> str:
    """Use immediate keypresses in a Windows terminal; input() elsewhere."""
    if os.name == "nt":
        import msvcrt
        while True:
            key = msvcrt.getwch().lower()
            if key in "wsadrq":
                print(key)
                return key
    return input("Command [w/s/a/d/r/q] + Enter: ").strip().lower()[:1]


def describe(solver: StewartKinematics, roll: float, pitch: float):
    """Check reach and calibrated limits before changing any hardware."""
    angles = solver.calculate_servo_angles(roll, pitch)
    command = solver.format_serial_command(angles)
    print(
        f"Roll={roll:+.1f} deg, pitch={pitch:+.1f} deg  "
        f"-> servos {tuple(round(x, 2) for x in angles)}  "
        f"-> {command!r}"
    )
    return command


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="Actually send commands to Arduino")
    parser.add_argument("--port", help="Override COM port from stewart_config.json")
    parser.add_argument("--step", type=float, default=1.0, help="Roll/pitch step in degrees")
    parser.add_argument(
        "--max-test-tilt", type=float, default=1.0,
        help="Maximum combined roll/pitch magnitude in degrees (default: 1.0)",
    )
    args = parser.parse_args()

    if not math.isfinite(args.step) or args.step <= 0:
        parser.error("--step must be a positive number")
    if not math.isfinite(args.max_test_tilt) or not 0 < args.max_test_tilt <= 2.0:
        parser.error("--max-test-tilt must be > 0 and <= 2 degrees for this initial test")

    solver = StewartKinematics.from_files()
    config_path = Path(__file__).resolve().parent / "stewart_config.json"
    with config_path.open(encoding="utf-8") as f:
        cal = json.load(f)
    port = args.port or cal["servos"]["port"]
    baud = int(cal["servos"].get("baud_rate", 9600))

    print("\nManual Stewart platform test")
    print("W: +pitch   S: -pitch   A: +roll   D: -roll   R: neutral   Q: quit")
    print("Directions are mathematical, not yet verified against camera axes.")
    print(f"Combined tilt limit: {args.max_test_tilt:g} deg")
    print("Remove the ball and support the platform. Stop if linkages bind.")
    describe(solver, 0.0, 0.0)

    ser = None
    try:
        if args.live:
            print("\nLIVE MODE: open Serial Monitor must be closed.")
            if input("Type ARM to enable hardware commands: ").strip() != "ARM":
                print("Not armed. No commands sent.")
                return
            import serial
            ser = serial.Serial(port, baudrate=baud, timeout=0.2, write_timeout=1)
            time.sleep(2.5)  # USB serial opening normally resets Arduino Mega
            if input("Press Enter to COMMAND the calibrated neutral pose (or type CANCEL): ").strip():
                print("Cancelled. No commands sent.")
                return
            ser.write(describe(solver, 0.0, 0.0))
            print("Neutral command sent. Inspect motion before further input.")
        else:
            print("\nDRY RUN only. Use --live when hardware is ready.")

        roll = 0.0
        pitch = 0.0
        while True:
            key = read_key()
            if key == "q":
                break
            next_roll, next_pitch = roll, pitch
            if key == "w":
                next_pitch += args.step
            elif key == "s":
                next_pitch -= args.step
            elif key == "a":
                next_roll += args.step
            elif key == "d":
                next_roll -= args.step
            elif key == "r":
                next_roll = next_pitch = 0.0
            else:
                continue

            if math.hypot(next_roll, next_pitch) > args.max_test_tilt + 1e-9:
                print("Blocked: exceeds initial combined tilt limit.")
                continue
            try:
                command = describe(solver, next_roll, next_pitch)
            except KinematicsError as exc:
                print("Blocked:", exc)
                continue

            if ser is not None:
                ser.write(command)
                # Read any immediately available reply without blocking the control loop.
                time.sleep(0.07)
                while ser.in_waiting:
                    reply = ser.readline().decode("utf-8", errors="replace").strip()
                    if reply:
                        print("Arduino:", reply)
            roll, pitch = next_roll, next_pitch
    except (KeyboardInterrupt, EOFError):
        print("\nStopped by user.")
    finally:
        if ser is not None:
            ser.close()
            print("Serial closed. Servos may HOLD their final commanded positions.")


if __name__ == "__main__":
    main()
