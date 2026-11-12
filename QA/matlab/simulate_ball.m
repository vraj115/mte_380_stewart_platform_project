function out = simulate_ball(strategy, wp, p)
%SIMULATE_BALL  Simulate the ball on the plate moving through waypoints.
%
%   out = simulate_ball(strategy, wp, p)
%
%   strategy  'position'  conventional position control: a PID drives the
%                         ball straight at each waypoint
%             'planned'   trajectory planning: accelerate, coast, then brake
%                         when the stopping distance v^2/(2 a_brake) reaches
%                         the distance left (same logic as waypoint_demo.py)
%   wp        N-by-2 waypoints in metres
%   p         parameters from default_params
%
%   Plant (both axes):  x'' = Kb_true * theta(t - delay)
%   "Push" below means tilt in degrees in the direction of wanted
%   acceleration:  pitch = push_x,  roll = -push_y.
%
%   out.t, out.x, out.y, out.vx, out.vy, out.speed   ball state
%   out.pitch, out.roll                              commanded tilt, deg
%   out.target                                       active waypoint
%   out.phase   0 feedback only, 1 accelerate, 2 coast, 3 brake, 4 parked
%   out.legs    one row per waypoint: [index, overshoot_m, time_s,
%               timed_out, arrival_time_s, peak_speed_m_s]

STILL   = 0.012;        % m/s, ball counts as at rest below this
ROLLING = 0.030;        % m/s, ball counts as rolling above this

dt   = p.dt;
nsub = 10;                              % plant integration steps per control step
h    = dt / nsub;
nd   = round(p.delay / dt);             % plant dead time in control steps
nm   = round(p.model_delay / dt);       % dead time the planner assumes

wp      = repmat(wp, p.laps, 1);
n_wp    = size(wp, 1);
n_steps = ceil(p.t_max / dt);

% ----- Plant state -----
pos = p.start;
vel = [0 0];
queue = zeros(max(nd, nm) + 1, 2);      % commands on their way to the ball

% ----- Estimator (what the camera gives the controller) -----
last_meas = pos;
v_hat = [0 0];

% ----- PID state -----
integral = [0 0];
prev_err = [NaN NaN];
d_filt   = [0 0];
saturated = false;

% ----- Waypoint state -----
idx = 1;
leg_start = pos;
leg_time  = 0;
dwell     = 0;
overshoot = 0;
peak_speed = 0;
legs = zeros(0, 6);

% ----- Planner state -----
move = [];
go_tilt = p.go_tilt;
still_time = p.stuck_wait;              % at rest at the start: plan at once
last_push = [0 0];

% ----- Log -----
L.t = zeros(n_steps, 1);  L.x = L.t;  L.y = L.t;  L.vx = L.t;  L.vy = L.t;
L.pitch = L.t;  L.roll = L.t;  L.phase = L.t;
L.target = zeros(n_steps, 2);
k_end = n_steps;

for k = 1:n_steps

    t = (k - 1) * dt;

    % ---------- Measurement and velocity estimate ----------
    meas = pos;
    if k > 1
        alpha = dt / (0.06 + dt);
        v_hat = v_hat + alpha * ((meas - last_meas) / dt - v_hat);
    end
    last_meas = meas;
    speed = norm(v_hat);

    % ---------- Waypoint bookkeeping ----------
    target = wp(idx, :);
    dist = norm(target - meas);
    leg_time = leg_time + dt;
    peak_speed = max(peak_speed, norm(vel));

    leg_vec = target - leg_start;
    leg_len = norm(leg_vec);
    if leg_len > 0.01
        % Overshoot: distance past the waypoint along the approach direction
        overshoot = max(overshoot, dot(meas - target, leg_vec) / leg_len);
    end

    if p.dwell <= 0
        arrived = dist < p.tolerance;                 % touch and go
    else
        if dist < p.tolerance && speed < 0.02
            dwell = dwell + dt;
        else
            dwell = 0;
        end
        arrived = dwell >= p.dwell;
    end

    timed_out = leg_time > p.leg_timeout;

    if arrived || timed_out

        legs(end + 1, :) = [idx, max(0, overshoot), leg_time, timed_out, t, peak_speed]; %#ok<AGROW>

        if idx == n_wp
            k_end = k - 1;
            break
        end

        leg_start = target;
        idx = idx + 1;
        target = wp(idx, :);
        dist = norm(target - meas);

        leg_time = 0;  dwell = 0;  overshoot = 0;  peak_speed = 0;
        move = [];
        if timed_out
            still_time = 0;
        else
            still_time = p.stuck_wait;
        end

        integral = [0 0];  prev_err = [NaN NaN];  d_filt = [0 0];
    end

    % ---------- Controller ----------
    phase = 0;

    switch lower(strategy)

        case 'position'
            % Conventional position control: PID on (waypoint - ball)
            err = target - meas;

            if any(isnan(prev_err))
                raw = [0 0];
            else
                raw = (err - prev_err) / dt;
            end
            prev_err = err;

            d_filt = d_filt + dt / (p.d_filter + dt) * (raw - d_filt);

            if ~saturated && p.ki > 0
                integral = integral + err * dt;
                lim = p.i_limit / p.ki;
                integral = max(-lim, min(lim, integral));
            end

            push = p.kp * err + p.ki * integral + p.kd * d_filt;

        case 'planned'

            if speed < STILL
                still_time = still_time + dt;
            elseif speed > ROLLING
                still_time = 0;
            end

            push = [0 0];
            decided = false;

            if ~isempty(move)

                elapsed = t - move.start;
                aim = move.aim;
                across = [-aim(2), aim(1)];

                % State when a command sent now reaches the ball: roll the
                % measurement forward through the commands still in transit
                pp = meas;  pv = v_hat;
                for j = nm:-1:1
                    pp = pp + pv * dt;
                    pv = pv + p.Kb * queue(j, :) * dt;
                end

                remaining = dot(target - pp, aim);
                v_along   = dot(pv, aim);

                % Gentle feedback back onto the straight line
                side = -(p.kp * dot(pp - target, across) + p.kd * dot(pv, across)) * across;

                move.top = max(move.top, dot(v_hat, aim));

                if move.phase == 1
                    if elapsed < move.accel_s
                        push = go_tilt * aim;                 % ACCELERATE
                        phase = 1;  decided = true;
                    else
                        move.phase = 2;
                    end
                end

                % The push should have freed the ball by now
                if ~decided && move.top < ROLLING && ...
                        elapsed > move.accel_s + p.model_delay + 0.3
                    go_tilt = min(0.9 * p.max_tilt, go_tilt + 0.5);
                    move = [];
                    still_time = p.stuck_wait;
                    decided = true;
                end

                if ~decided && move.phase == 2

                    % Start braking when the stopping distance reaches
                    % the distance left:  d_stop = v^2 / (2 a_brake)
                    d_stop = v_along^2 / (2 * p.brake_accel);

                    if v_along > 0 && d_stop >= remaining
                        move.phase = 3;
                    elseif elapsed > move.accel_s + p.model_delay && v_along < 0.01
                        move = [];  still_time = 0;           % stopped short
                        push = side;  phase = 2;  decided = true;
                    else
                        hold = max(-1, min(1, 1.5 * (move.peak - v_along) / p.Kb));
                        if elapsed < move.accel_s + p.model_delay
                            hold = 0;
                        end
                        push = hold * aim + side;             % COAST
                        phase = 2;  decided = true;
                    end
                end

                if ~decided && move.phase == 3

                    % BRAKE: deceleration that stops the ball exactly at
                    % the waypoint from here,  a = v^2 / (2 d)
                    if v_along < 0.012
                        move = [];  still_time = 0;
                        push = [0 0];
                    else
                        limit = 0.8 * p.max_tilt * p.Kb;
                        if remaining > 0.002
                            decel = min(limit, v_along^2 / (2 * remaining));
                        else
                            decel = limit;
                        end
                        push = -decel / p.Kb * aim + side;
                    end
                    phase = 3;  decided = true;
                end
            end

            if ~decided

                if dist < p.tolerance && speed < STILL
                    push = -p.kd * v_hat;                     % PARKED
                    phase = 4;

                elseif still_time > p.stuck_wait && dist > 1e-4

                    % At rest away from the waypoint: plan a move
                    accel = p.Kb * go_tilt;
                    brake = p.brake_accel;

                    % Highest speed that still leaves room to stop:
                    % dist = v^2/(2 accel) + v^2/(2 brake)
                    peak = min(p.cruise, sqrt(2 * dist * accel * brake / (accel + brake)));

                    move = struct('start', t, 'aim', (target - meas) / dist, ...
                        'peak', peak, 'accel_s', peak / accel, 'phase', 1, 'top', 0);

                    push = go_tilt * move.aim;
                    phase = 1;

                else
                    % Still rolling with no move planned: plain feedback
                    push = p.kp * (target - meas) - p.kd * v_hat;
                end
            end

        otherwise
            error('Unknown strategy "%s".', strategy);
    end

    % ---------- Platform limits ----------
    wanted = norm(push);
    if wanted > p.max_tilt
        push = push * p.max_tilt / wanted;
    end
    saturated = wanted > p.max_tilt;

    step = push - last_push;
    max_step = p.tilt_rate * dt;
    if norm(step) > max_step
        push = last_push + step * max_step / norm(step);
    end
    last_push = push;

    % ---------- Dead time, then the ball ----------
    queue = [push; queue(1:end - 1, :)];
    applied = queue(nd + 1, :);

    for s = 1:nsub
        vel = vel + p.Kb_true * applied * h;
        pos = pos + vel * h;
    end

    % ---------- Log ----------
    L.t(k) = t;  L.x(k) = meas(1);  L.y(k) = meas(2);
    L.vx(k) = vel(1);  L.vy(k) = vel(2);
    L.pitch(k) = push(1);  L.roll(k) = -push(2);
    L.phase(k) = phase;  L.target(k, :) = target;
end

keep = 1:k_end;

out.t = L.t(keep);        out.x = L.x(keep);        out.y = L.y(keep);
out.vx = L.vx(keep);      out.vy = L.vy(keep);
out.speed = hypot(out.vx, out.vy);
out.pitch = L.pitch(keep);  out.roll = L.roll(keep);
out.phase = L.phase(keep);  out.target = L.target(keep, :);
out.legs = legs;
out.strategy = strategy;
out.finished = size(legs, 1) == n_wp;

end
