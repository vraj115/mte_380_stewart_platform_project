# Information for the report: "1. Eliminating Ball Position Overshoot Through Trajectory Planning"

Everything needed to turn the planned ("will be simulated...") sections of the write-up
into results. All MATLAB numbers below come from running the scripts in `QA/matlab` in
MATLAB R2026b. All platform numbers come from the logs named next to them.

## 1. Read this first: what actually ran on the platform

The run that worked, with the slider gains Kp 27, Ki 2, Kd 20.5, was **not** driven by the
accelerate / coast / brake planner. The log shows this directly:

| Phase label in `waypoint_run_20261112_163556.csv`, 65.9-237.1 s | Share of time |
|---|---|
| CATCH (position feedback: `kp * error - kd * velocity`) | 99.6% |
| BRAKE | 0.2% |
| ACCELERATE | 0.1% |

The reason: the planner only plans a move when the ball is at rest, and with touch-and-go
waypoints (`--dwell 0`) the ball never comes to rest. It rolls continuously at about
117 mm/s, so the controller stays in its feedback mode. What worked on the platform is
therefore **position (PD) control with touch-and-go waypoints**, which is the report's
"conventional position control", not its "trajectory-planned control".

What this means for the report:

- The MATLAB section can be written as planned: the simulation does show trajectory
  planning removing overshoot (section 3).
- The physical section cannot claim that trajectory planning reduced overshoot on the
  platform. The honest account is in section 5, and section 8 lists one short experiment
  that would give a real hardware comparison.
- Ki had no effect in that run: the planner's feedback mode has no integral term.

## 2. Parameters to quote

| Quantity | Value | Where it comes from |
|---|---|---|
| Ball model (ideal) | x'' = (3/5) g theta = 0.10273 m/s^2 per degree | derivation in the write-up |
| Ball gain measured on the platform | 0.092 m/s^2 per degree (90% of ideal) | fit to `waypoint_run_20261112_163556.csv`, 13,020 samples, R^2 = 0.85 |
| Dead time (camera + serial + servos) | 0.23 s | same fit |
| Plate level error at neutral | pitch -0.69 deg, roll +0.48 deg | same fit |
| Gain and dead time in earlier, shorter logs | 0.040-0.089 m/s^2 per degree, 0.17-0.30 s | logs of 15:39-16:01 on 12 Nov; poorer fits (R^2 0.27-0.81) |
| Feedback gains | Kp 27 deg/m, Ki 2 deg/(m s), Kd 20.5 deg/(m/s) | sliders, final values |
| Tilt limit / slew limit | 5 deg combined / 60 deg/s | `kinematics_config.json`, `waypoint_demo.py` |
| Control rate | 30 Hz | camera frame rate |
| Diamond | corners 80 mm from the centre; sides 113 mm | `--size 0.08` |
| Target circle | 25 mm radius, judged on the ball's centre | `--tolerance 0.025` |
| Planner: accelerating tilt | 3 deg (0.31 m/s^2 on the ideal model) | `--go-tilt` |
| Planner: planned braking | 0.12 m/s^2 (1.17 deg) | `--brake-accel` |
| Planner: cruise speed | 0.08 m/s | `--cruise-speed` |
| Plate | 300 mm diameter; ball leaves at 150 mm from centre | `stewart_config.json` |

With delay, the transfer function in the write-up becomes

    G(s) = X(s) / Theta(s) = K e^(-tau s) / s^2

with K = 5.886 m/s^2 per radian (0.10273 per degree) for the ideal ball and tau = 0 in the
derivation; on the platform K is about 5.27 per radian (0.092 per degree) and tau is about
0.23 s. The write-up's `5.886 / s^2` is correct only with theta in radians; say so, because
every other number in the report is in degrees.

Stability of the feedback loop with the final gains on the measured plant (Kp and Kd acting,
velocity filter 0.06 s): crossover 2.3 rad/s, **phase margin about 22 deg, gain margin 2.1x**.
For comparison the earlier setting Kp 30, Kd 10, which orbited, has about 4 deg. On the ideal
plant with no dead time the same gains have about 50 deg. This is the quantitative reason
the write-up gives for not relying on gain tuning alone: the dead time caps the usable gain.

## 3. MATLAB results (for "MATLAB Simulation and Performance Evaluation")

Both strategies use the same gains (27 / 2 / 20.5), tilt limits, start point and waypoints.
A waypoint counts as reached when the ball's centre has stayed inside the 25 mm circle,
slower than 20 mm/s, for 0.25 s. Overshoot is the largest distance past the waypoint along
the direction of approach.

### Ideal model (the write-up's model, no dead time) - `run_comparison.m`

| | Position control | Trajectory planned |
|---|---|---|
| Mean overshoot | 10.0 mm | 0.0 mm |
| Worst overshoot | 11.8 mm | 0.0 mm |
| Lap time (6 waypoints) | 12.6 s | 11.0 s |
| Peak ball speed | 74-109 mm/s | 81-84 mm/s |
| Peak commanded tilt | 3.22 deg | 3.00 deg |

Per waypoint, position control overshoots by 7.2, 11.8, 11.0, 11.0, 11.0 and 8.0 mm; the
planner by 0.0 mm at all six.

### Measured plant (0.092 m/s^2 per degree, 0.23 s dead time) - same script

| | Position control | Trajectory planned |
|---|---|---|
| Mean overshoot | 33.8 mm | 11.3 mm |
| Worst overshoot | 37.6 mm | 26.9 mm |
| Lap time | 19.3 s | 14.6 s |
| Peak commanded tilt | 3.21 deg | 4.00 deg |

With dead time the planner still helps but no longer removes overshoot: three of the six
waypoints are reached with 0.0 mm and three with 16-27 mm.

### Effect of the planned acceleration and braking - `sweep_planner_settings.m`

- **Ideal model:** overshoot is 0.0 mm for every combination tried (accelerating tilt
  2-4 deg, braking 0.04-0.30 m/s^2). Only the lap time changes: 13.3 s at the gentlest
  setting, falling to 10.1 s at 4 deg and 0.25 m/s^2. At 0.30 m/s^2 the lap time rises again
  to about 12 s; I did not trace why (most likely extra corrective moves at each waypoint).
- **Measured plant:** overshoot is 0.0 mm only when braking is gentle: 0.06 m/s^2 or less
  (0.08 or less when accelerating at 4 deg). Above that it is 5-12 mm. Gentle braking starts
  earlier, which leaves room for the 0.23 s of dead time.
- **Compromise to report:** on the measured plant, accelerating at 3 deg and braking at
  0.06 m/s^2 (0.6 deg) gave zero overshoot with a 13.3 s lap; accelerating at 4 deg and
  braking at 0.08 m/s^2 gave zero overshoot with a 12.6 s lap. Position control with the same
  gains gave 33.8 mm and 19.3 s.

### Figures (all in `QA/figures`)

| Report figure | File | What it shows |
|---|---|---|
| Figure 1 | `fig1_xy_path_ideal.png` | X-Y path, both strategies, ideal model |
| Figure 2 | `fig2_position_ideal.png` | x and y against time, dotted = active waypoint |
| Figure 3 | `fig3_speed_ideal.png` | Ball speed against time, with the planner's cruise speed |
| Figure 4 | `fig4_tilt_ideal.png` | Commanded pitch and roll, with the 5 deg limit |
| Optional | `fig1...fig4_*_measured.png` | The same four on the measured plant |
| Figure 5 | `fig5_planner_sweep.png` | Overshoot and lap time against planned braking |
| Figure 6 | `fig6_experiment_xy.png` | Measured ball path on the platform next to the simulation |
| Figure 7 | `fig7_experiment_time.png` | 20 s of the measured run: position, speed, tilt |

What to point out in each:

- **Figure 1:** position control swings wide of every corner; the planned path stays on the
  diamond and stops at each corner.
- **Figure 3:** position control peaks above 100 mm/s and is still moving when it reaches the
  waypoint. The planned speed is a flat-topped profile capped near 80 mm/s that returns to
  zero at each waypoint.
- **Figure 4:** the planner's tilt is a positive pulse (accelerate) followed by a negative
  one (brake) on each leg. Position control's tilt is one large pulse that decays and
  reverses late. Neither reaches the 5 deg limit.

## 4. Corrections to the current write-up

1. **"Braking begins halfway through the displacement."** True only for equal acceleration
   and braking. The implementation accelerates at 3 deg (0.31 m/s^2) and plans braking at
   0.12 m/s^2, so with no coasting the acceleration phase covers 28% of the distance and
   braking 72%. On a 113 mm side with the 0.08 m/s speed cap: 10 mm accelerating, 76 mm
   coasting, 27 mm braking.
2. **Three phases, not two.** The implementation adds a coast phase at a capped speed
   between accelerating and braking.
3. **Braking is closed loop.** Braking starts when `d_stop = v^2 / (2 a_brake)` reaches the
   distance left, as the write-up says. From then on the tilt is recalculated every frame as
   `a = v^2 / (2 d)` from the measured speed and distance.
4. **Dead time.** The write-up's model has none. The implementation projects the ball's
   position and speed one dead time ahead, using the commands already sent, before applying
   the stopping-distance rule.
5. **Units of the transfer function** (see section 2).
6. **Tense.** "Will be simulated / will be tested" can now be past tense, using sections 3
   and 5.

## 5. Platform results (for "Physical Implementation and Validation")

### The run that worked: position feedback, touch-and-go

`waypoint_run_20261112_163556.csv`, 65.9-237.1 s (171 s), Kp 27-28.5, Ki 2, Kd 20-20.5,
diamond 80 mm, 25 mm circles, waypoint reached the moment the ball's centre enters the circle.
The simulation column is the same configuration on the measured plant
(`compare_with_experiment.m`).

| | Platform | Simulation |
|---|---|---|
| Waypoints reached | 91 in 171 s | 34 (six laps) |
| Mean time per waypoint | 1.83 s | 1.14 s |
| Mean overshoot past a waypoint | 27.1 mm | 26.8 mm |
| Median overshoot | 25.7 mm | 27.5 mm |
| Worst overshoot | 71.0 mm | 39.3 mm |
| Mean speed on arrival | 121 mm/s | 141 mm/s |
| Median ball speed | 117 mm/s | 124 mm/s |
| Furthest from centre | 147 mm (edge is 150 mm) | 103 mm |
| RMS commanded tilt | 3.07 deg | 2.77 deg |
| Time in feedback-only mode | 99.6% | 97% |

The last 31 s of that run used exactly Kp 27, Kd 20.5: 16 waypoints, 1.6 s each on average,
mean overshoot 20.9 mm, worst 56.1 mm.

Points for the discussion:

- The simulation reproduces the typical overshoot (27 mm in both) and ball speed, which
  supports the identified model (0.092 m/s^2 per degree, 0.23 s).
- It does not reproduce the worst cases. The real ball came within 3 mm of the plate edge;
  the simulated one stayed 47 mm inside it. The simulation has no level error, no uneven
  ball and no camera noise.
- The real ball takes 60% longer per waypoint. Figure 6 suggests part of the reason: its
  path often loops past a 25 mm circle and has to come round again.
- The ball arrives at each waypoint at about 120 mm/s. This is exactly the mechanism the
  write-up describes: residual velocity at the waypoint causes the overshoot.

### Runs where the planner's phases did execute

- `waypoint_run_20261112_160419.csv` (rest-at-waypoint, gains not recorded): ACCELERATE 16%,
  COAST 15%, BRAKE 5%, feedback 62% of the time. 15 waypoints in 507 s, median 11.2 s each,
  mean overshoot 39.9 mm. The planner ran but performed worse than the feedback-only run.
- `waypoint_run_20261112_150837.csv` (first live run, 15 mm circles): 91% of the time in
  feedback mode; the ball reached 218 mm/s and 124 mm from the centre.

### Why the planner underperformed on the platform

1. **The ball rarely comes to rest**, and the planner starts each move from rest. When the
   ball keeps rolling, control falls back to position feedback.
2. **A resting ball ignores small tilts.** In the balancing logs a stationary ball almost
   never started rolling within 0.5 s for net tilts under about 2 deg. This is why the
   accelerating tilt is 3 deg. The team attributes it to the ball's uneven mass; servo
   backlash was not ruled out.
3. **Dead time of 0.23 s.** At 0.08 m/s the ball travels 18 mm before a command takes
   effect, comparable to the 27 mm planned braking distance.
4. **The plate is not level**, and the error changed between sessions (up to about 2 deg on
   12 Nov). A 1 deg error is 0.09 m/s^2, close to the whole planned braking deceleration.
5. **Model gain.** The planner used the ideal 0.10273; the plate measured 0.092.

### Balancing at the centre (context for the opening paragraph)

`pid_run_20261112_162906.csv`, Kp 31.5, Ki 4, Kd 20: the ball held a median 7-14 mm from the
centre across four stretches of 13-65 s.

## 6. Suggested wording for the conclusion

> In simulation with the derived model, trajectory planning removed waypoint overshoot
> entirely (0.0 mm against 10.0 mm for position control with the same gains) and shortened the
> lap from 12.6 s to 11.0 s. With the plant measured on the platform, which adds 0.23 s of
> dead time, planning reduced mean overshoot from 33.8 mm to 11.3 mm, and to zero when the
> planned braking was reduced to 0.06 m/s^2.
>
> On the physical platform the planner could not be validated in the same way. The ball
> seldom came to rest between waypoints, so the planned accelerate and brake phases ran
> for under 1% of the successful run, and the diamond was traced by position feedback with
> touch-and-go waypoints. That run reached 91 waypoints in 171 s with a mean overshoot of
> 27 mm, which the simulation of the same configuration reproduces (27 mm). The main
> obstacles to planned motion on the hardware were the loop's dead time, the ball's
> reluctance to start rolling at small tilts, and a plate level error of up to 2 deg.

## 7. Running the MATLAB scripts

Open MATLAB in `QA/matlab` and run:

| Script | Produces |
|---|---|
| `run_comparison` | Figures 1-4 for the ideal and the measured plant, and the per-waypoint tables |
| `sweep_planner_settings` | Figure 5 and its table |
| `compare_with_experiment` | Figures 6-7 and the platform-against-simulation table |

Supporting files: `default_params.m` (every parameter, with units), `simulate_ball.m` (the
plant and both controllers), `diamond_waypoints.m`, `light_figure.m`. No toolboxes are
needed. To change the gains or the planner settings, edit `default_params.m`.
`QA/data` holds the platform log used by `compare_with_experiment`.

`simulate_ball.m` follows the logic of `waypoint_demo.py` but leaves out what the report's
model does not contain: ball sticking, level error, level learning and camera noise.

## 8. What is still missing, and one experiment that would fill it

There is no platform log of the two strategies under the same conditions, which is what the
write-up promises. Both of these take about two minutes each:

    python waypoint_demo.py --live --log --dwell 0.25 --kp 27 --ki 2 --kd 20.5

Run it once as it starts (planner on), then again pressing **P** before SPACE (position
control). With `--dwell 0.25` the ball must rest at each waypoint, so the planner's phases
run and overshoot can be measured the same way as in MATLAB. Pass the level trim printed at
the end of the previous run (`--trim-roll`, `--trim-pitch`) to both.

Point `LOG_FILE` and `T_WINDOW` in `compare_with_experiment.m` at each new log to get its
overshoot and timing table.
