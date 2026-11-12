# Session notes (10-12 Nov 2026)

A summary of the working session that produced the trajectory and waypoint code
on this branch. Give this file to a new session (or a new teammate) first: it says
what exists, what was measured, what was tried, and what is still open.

`CHANGES.md` covers the earlier balancing fixes and is still accurate, except for
its line "Nothing here has been run on the real platform": the balancing code has
since been run many times (see the `pid_run_*.csv` logs).

## The project

MTE 380 ball-on-plate: a 3-servo Stewart platform (Arduino Mega + PCA9685) tilts a
300 mm plate, a camera tracks an orange ping-pong ball, Python closes the loop.

| File | Purpose |
|---|---|
| `pid_controller.py` | Balances the ball at the centre. Tuned and working. |
| `kinematics.py`, `ball_detection.py`, `stewart_cal.py` | Inverse kinematics, ball detection, calibration. |
| `servo_controller/servo_controller.ino` | Arduino sketch: 115200 baud, decimal angles. |
| `analyze_log.py` | Measures delay, gain, axis rotation and level error from `pid_run_*.csv`. |
| `trajectory_demo.py` | Smooth shapes (circle, figure 8, star, square, triangle), feedforward + PID. |
| `trajectory_theory.html` | Theory page for the above, with an interactive simulator. Works offline. |
| `waypoint_demo.py` | Waypoint planner built on the team's stopping-distance model. **The current focus.** |
| `waypoint_report/` | Figures from `python waypoint_demo.py --report` (simulated). |
| `pid_run_*.csv`, `waypoint_run_*.csv` | Logs from the real platform. |

`stewart_config.json` on this branch uses camera index 0 and port COM10 (the lab
machine used for these runs). `main` has camera 1 and COM5.

## What was measured on the real platform

| Quantity | Value | Source |
|---|---|---|
| Ball acceleration per degree of tilt, rolling | 0.06 to 0.11 m/s^2 (ideal hollow ball: 0.10273) | `pid_run` logs of 10-12 Nov |
| Same, from the planner's timed pushes | about 0.065 m/s^2 per degree | `waypoint_run_20261112_150837.csv` |
| Dead time (camera + serial + servos) | 0.2 to 0.3 s | `analyze_log.py`, log fits |
| Plate level error at neutral | changes between sessions; about 1.3 deg roll on 12 Nov | planner's level estimate |
| Servo resolution | 2.5 PWM ticks per servo degree, about 0.28 deg of plate tilt per tick | `servo_controller.ino` |

Two facts about the ball matter more than anything else:

1. **At rest it ignores small tilts.** In the balancing logs a stationary ball almost
   never started rolling within 0.5 s for net tilts under about 2 deg, and often sat
   for several seconds. The team attributes this to the ball's uneven mass. Servo
   backlash was not ruled out; the logs cannot tell the two apart.
2. **It does not always stay at rest.** In the first live planner run the ball was
   still rolling 91% of the time. How sticky it is varies from run to run.

## What was tried, in order

1. **Balancing stabilised** (`CHANGES.md`): gains were too high for the dead time.
   Defaults are now `kp 20, ki 8, kd 25`.
2. **Slow hunting near the centre** was diagnosed from `pid_run_20261110_152956.csv`:
   the ball sticks off-centre, the integral ramps for about 10 s, the ball breaks free
   and overshoots by up to 70 mm. A fix was proposed (slow integral for level error
   only, plus a fast "breakaway" term that is dropped once the ball rolls) but **was
   not implemented** in `pid_controller.py`.
3. **`trajectory_demo.py`** tracked shapes well in simulation (3-5 mm RMS) and badly
   on the plate. Reason: its smooth plans ask for 0.5-1 deg of tilt, inside the range
   a resting ball ignores. Slowing the lap makes the tilts smaller and the result
   worse. Kept for the theory page; not the way forward for this ball.
4. **`waypoint_demo.py`** was written from the team's model: `x'' = (3/5) g theta`,
   stopping distance `d_stop = v^2 / (2 a_brake)`, tilt = wanted acceleration / gain.

## How `waypoint_demo.py` works (version on this branch)

Each move starts from rest and has three phases:

- **ACCELERATE**: a timed tilt of 3 deg (`--go-tilt`) toward the waypoint. Above the
  sticking threshold on purpose. Raised by 0.5 deg if the ball did not move.
- **COAST**: plate level, cruise speed 0.08 m/s.
- **BRAKE**: starts when `d_stop` reaches the distance left; tilt is then set from
  `a = v^2 / (2 d)` using speed and distance projected one dead time ahead.

Then: **PARKED** when the ball is slow inside the target circle (only damping is
applied); another short move if it stopped outside; **CATCH** (plain PD, no speed
limit) while the ball is rolling with no move planned. It also learns the plate's
level error while the ball rolls and prints trim values on quit.

Defaults on this branch: shape diamond, size 0.08 m (centre to corner), target circle
0.025 m, ball must rest in the circle for 0.25 s, model gain 0.10273, dead time 0.25 s.

```
python waypoint_demo.py --sim           # simulated sticking ball, no hardware
python waypoint_demo.py --report        # regenerate waypoint_report/ figures
python waypoint_demo.py --live --log --trim-roll 1.3 --trim-pitch -0.3
```
Keys: SPACE start/pause, P planner on/off (off = PID stepping between waypoints),
1-4 diamond/square/triangle/star, R neutral, Q quit.

Simulated results for this version (108 legs, sticking ball): overshoot past a
waypoint 0.8 mm mean, 12 mm worst, 2.9 s per leg, no timeouts. Plain PID in the same
kind of simulation: 20 to 28 mm mean. On the ideal ball: 0 mm against 28 mm.

**The simulation is only a rough stand-in.** Its ball sits still far more than the
real one under PID (about 87% of the time against 24-53%). Treat simulated numbers as
a check that the logic works, not as a prediction.

## What happened on the real platform, and what is still open

- First live run (`waypoint_run_20261112_150837.csv`, 15 mm circles, rest required):
  the ball spent 91% of the time in CATCH, reached 218 mm/s and 124 mm from the centre.
  CATCH has no speed limit, so on a long crossing the ball accelerates the whole way.
- A touch-and-go variant (10-13 mm circles, move on as soon as the ball's centre
  enters) made this worse on the plate: the ball overshot and fell off.
- A later variant replaced CATCH with a speed-limited approach (cap 0.07 m/s), added an
  edge guard past 100 mm, and set the gain to 0.075 and dead time to 0.30 s. In
  simulation it cut top speed from 123-205 to 105-129 mm/s but still ran up to about
  30 mm past a corner. It was not confirmed on the plate. One live log exists from that
  period (`waypoint_run_20261112_153957.csv`) and **has not been analysed**.
- The team then asked to go back to the 25 mm rest-at-waypoint version, which is what
  `waypoint_demo.py` is now. The speed-limited variant is in the git history of this
  branch (the commit before the one that restored this version).

Open problems, most important first:

1. **CATCH in the current version still has no speed limit**, so the ball-falls-off
   failure on long crossings is still possible. The speed-limited approach from the
   previous commit is the starting point for fixing it.
2. **Level trim.** Starting without `--trim-roll` / `--trim-pitch` costs the first
   seconds of every run. Use the values the demo prints on quit.
3. **Corners are close to the edge.** At size 0.08 the corners are 70 mm from the plate
   edge. Size 0.06 leaves more room.
4. A velocity-based edge guard was tried and removed: with the dead time it made a
   free-rolling ball oscillate.
5. The breakaway fix for plain balancing (item 2 above) is still not implemented.

## Working preferences that came up

- Simulate first (`--sim`), then run live with `--log`, then analyse the log before
  changing anything. The real logs changed the diagnosis twice.
- Explain results in plain language and say clearly what was only tested in simulation.
- Small, direct requests (circle size, dwell time, shape size) should just be done.

## Git

Branch `pid-stability-fixes` on `vraj115/mte_380_stewart_platform_project`, based on
`main` at `faf3470`. No pull request has been opened. `__pycache__/` and header-only
logs are deliberately not committed.
