# What changed and how to run it

This folder is a modified copy of `mte_380_stewart_platform_project` (commit `faf3470`).
Nothing here has been run on the real platform. It was tested by running the real
`pid_controller.py` against a simulated camera, Arduino and ball built from the
measurements below. The Arduino sketch has not been compiled.

## Why the ball spiralled outward

Measured from the five `pid_run_20261110_*.csv` logs (`python analyze_log.py` reproduces this):

| Quantity | Measured | Meaning |
|---|---|---|
| Axis rotation | +0.1 deg | Camera axes, roll/pitch signs and servo order are all correct |
| Dead time | 0.20 s | Command to ball response (camera + serial + servos) |
| Gain | 0.070 m/s^2 per commanded degree | 68% of the ideal rolling-ball value |
| Bias | +0.10 m/s^2 toward +x at neutral | Plate is about 1 deg off level at neutral |

The kinematics are correct. The loop was unstable because the gains were too high for
0.2 s of dead time:

- The gains in the logs were about `kp 75, ki 10, kd 50` (and `kp 30, kd 12` in the first run).
  With the measured delay, the 10.6 Hz loop and the 0.15 s derivative filter, `75/10/50`
  has a phase margin of about -13 deg, so any small motion grows into an orbit.
- The code defaults (`kp 40, kd 0`) have no damping at all and are also unstable.
- The 12 deg/s tilt slew limit and the 3 deg tilt limit were active most of the time
  (tilt at its limit in over half the samples), which adds more lag as the orbit grows.
- Whole-degree servo commands are about 0.7 deg of plate tilt per step.
- One missed ball detection paused balancing and sent the plate to neutral.

## What was changed

`pid_controller.py`
- Default gains `kp 20, ki 8, kd 25` (about 34 deg phase margin on the measured plant;
  tolerates roughly 2.8x more or 3.7x less plant gain).
- Control runs on every camera frame (default up to 30 Hz, was about 10.6 Hz).
- Derivative filter 0.08 s (was 0.15 s), settable with `--d-filter`.
- Tilt slew limit 60 deg/s (was 12), max tilt default 5 deg (the old default of 8 was
  rejected by its own check).
- Integral term with real anti-windup, limited to 2.5 deg, so it can cancel the level error.
- `--trim-roll` / `--trim-pitch`, and the tilt needed to hold the ball is printed on exit.
- Ball may go undetected for 0.4 s before balancing pauses (`--lost-timeout`).
- Servo angles sent in tenths of a degree; the Arduino must acknowledge neutral before
  balancing is allowed, with a clear message if the old sketch is still loaded.
- Ball is detected once per frame instead of twice; loop rate and I-term shown on screen.
- Log has four extra columns: `dt_s, saturated, i_roll_deg, i_pitch_deg`.

`servo_controller/servo_controller.ino`
- 115200 baud (was 9600) and decimal angles. Whole-degree commands give exactly the same
  pulse as before, so the neutral calibration is unchanged.

`stewart_config.json`
- `baud_rate` 115200.
- `max_angles_deg` changed from `[42, 51, 59]` to `[40, 50, 59]` to match the limits
  hard-coded in the sketch. Before, a command of 41-42 on servo 1 or 51 on servo 2 would
  have been rejected by the Arduino and paused the run.

`kinematics.py`
- `format_serial_command(angles, decimals=0)`; fixed the self-test, which was failing.

`stewart_cal.py` - baud 115200. `ball_detection.py` - `draw_detection` can reuse a detection.
`analyze_log.py` - new; measures delay, gain, axis rotation and level error from logs.

## Steps to run

1. **Upload the sketch.** Open `servo_controller/servo_controller.ino` in the Arduino IDE
   and upload it to the Mega. In the Serial Monitor at **115200** baud you should see
   `READY: ... (115200 baud, decimals ok)`. Close the Serial Monitor afterwards.
2. **Activate the environment** in this folder: `conda activate stewart_platform_project`.
3. **Check the camera without motors:** `python pid_controller.py`
   The ball position should read near zero at the centre, +x to the right, +y up.
4. **Check directions without the ball** (optional, they were correct in the logs):
   `python manual_platform_control.py --live`
5. **Balance:** `python pid_controller.py --live --log`
   Type `ARM`, then `HOME`. Wait for "Arduino acknowledged neutral". Put the ball near the
   centre and press SPACE. R returns to neutral, Q quits.
6. **Add the trim.** The plate is about 1 deg off level, so the first seconds will drift
   toward +x until the integral term catches up. On exit the program prints the trim it
   found; from the old logs it should be close to:
   `python pid_controller.py --live --log --trim-pitch -1.5 --trim-roll -0.1`
7. **Re-measure:** `python analyze_log.py` and compare delay and gain with the table above.

## Tuning from here

- Ball oscillates or orbits with growing size: lower `--kp` (try 15) before anything else.
- Ball is sluggish but steady: raise `--kp` in steps of 5, keeping `--kd` about equal to it.
- Plate buzzes or twitches: raise `--d-filter` to 0.10-0.12 or lower `--kd`.
- Ball sits off-centre for a long time: set the trim, or raise `--ki` (keep it under half of kp).
- The on-screen "Loop" rate should read close to 30 Hz. If it reads 15, the camera is only
  delivering 15 or 30 fps unevenly; the dead time is what limits how much gain is usable.
