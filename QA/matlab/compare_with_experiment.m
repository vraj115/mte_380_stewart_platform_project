%COMPARE_WITH_EXPERIMENT  Platform log against the simulation (Figures 6-7).
%
%   Run from this folder:   compare_with_experiment
%
%   Reads a log written by "python waypoint_demo.py --live --log", picks the
%   part recorded with the chosen gains, measures overshoot and travel time
%   per waypoint, and simulates the same configuration on the measured plant.
%
%   The configuration that ran on the platform was:
%     * touch and go: a waypoint is reached when the ball's centre enters
%       the 25 mm target circle (dwell = 0)
%     * gains kp = 27 to 27.5, ki = 2, kd = 20 to 20.5 (set with the sliders)
%   In that run the ball never came to rest, so the planner's
%   accelerate / coast / brake phases did not run and the motion came from
%   position feedback (kp and kd) alone. See report_information.md.

clear; close all; clc;

LOG_FILE = fullfile('..', 'data', 'waypoint_run_20261112_163556.csv');
T_WINDOW = [65.9 237.1];        % s, the part recorded with the gains below
SIZE     = 0.08;

% ----- Load the log -----
T = readtable(LOG_FILE, 'TextType', 'string');
T = T(T.elapsed_s >= T_WINDOW(1) & T.elapsed_s <= T_WINDOW(2), :);

fprintf('Log: %s\n', LOG_FILE);
fprintf('Window %.1f to %.1f s, %d samples\n', T_WINDOW(1), T_WINDOW(2), height(T));
fprintf('Gains in the window: kp %.1f-%.1f, ki %.1f-%.1f, kd %.1f-%.1f\n', ...
    min(T.kp), max(T.kp), min(T.ki), max(T.ki), min(T.kd), max(T.kd));

phases = unique(T.phase);
for i = 1:numel(phases)
    fprintf('  phase %-12s %5.1f %% of the time\n', phases(i), ...
        100 * mean(T.phase == phases(i)));
end

exp_legs = experiment_legs(T);

% ----- Simulate the same configuration -----
p = default_params('measured');
p.dwell = 0;                    % touch and go, as on the platform
p.kp = 27.5;  p.ki = 2;  p.kd = 20;
p.laps = 6;
p.t_max = 120;

sim = simulate_ball('planned', diamond_waypoints(SIZE), p);
sim_phase_feedback = 100 * mean(sim.phase == 0);

% Overshoot in the simulation, measured the same way as in the log
S = table(sim.t, sim.x, sim.y, sim.target(:, 1), sim.target(:, 2), sim.speed, ...
    'VariableNames', {'elapsed_s', 'ball_x_m', 'ball_y_m', 'target_x_m', 'target_y_m', 'speed_m_s'});
sim_legs = experiment_legs(S);

% ----- Table -----
fprintf('\n                              Experiment    Simulation\n');
fprintf('Waypoints reached            %10d %13d\n', size(exp_legs, 1), size(sim_legs, 1));
fprintf('Mean time per waypoint (s)   %10.2f %13.2f\n', mean(exp_legs(:, 1)), mean(sim_legs(:, 1)));
fprintf('Mean overshoot (mm)          %10.1f %13.1f\n', mean(exp_legs(:, 2)), mean(sim_legs(:, 2)));
fprintf('Median overshoot (mm)        %10.1f %13.1f\n', median(exp_legs(:, 2)), median(sim_legs(:, 2)));
fprintf('Worst overshoot (mm)         %10.1f %13.1f\n', max(exp_legs(:, 2)), max(sim_legs(:, 2)));
fprintf('Mean arrival speed (mm/s)    %10.0f %13.0f\n', mean(exp_legs(:, 3)), mean(sim_legs(:, 3)));
fprintf('Median ball speed (mm/s)     %10.0f %13.0f\n', median(T.speed_m_s) * 1000, median(sim.speed) * 1000);
fprintf('Furthest from centre (mm)    %10.0f %13.0f\n', ...
    max(hypot(T.ball_x_m, T.ball_y_m)) * 1000, max(hypot(sim.x, sim.y)) * 1000);
fprintf('RMS tilt (deg)               %10.2f %13.2f\n', ...
    sqrt(mean(T.roll_deg.^2 + T.pitch_deg.^2)), sqrt(mean(sim.roll.^2 + sim.pitch.^2)));
fprintf('Feedback-only share of time  %9.0f%% %12.0f%%\n', ...
    100 * mean(T.phase == "CATCH"), sim_phase_feedback);

% ----- Figures -----
c_exp = [0.91 0.47 0.10];
c_sim = [0.04 0.43 0.60];
c_path = [0.35 0.35 0.35];

wp = diamond_waypoints(SIZE);
path_xy = [wp(1:5, :)] * 1000;
th = linspace(0, 2 * pi, 60);

out_dir = fullfile('..', 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% Figure 6: measured and simulated paths
f6 = light_figure([100 100 1000 480]);
titles = {'Platform (measured)', 'Simulation (measured plant)'};
for a = 1:2
    subplot(1, 2, a); hold on; grid on; axis equal;
    plot(150 * cos(th), 150 * sin(th), '-', 'Color', [0.8 0.8 0.8]);
    if a == 1
        plot(T.ball_x_m * 1000, T.ball_y_m * 1000, 'Color', c_exp, 'LineWidth', 0.8);
    else
        plot(sim.x * 1000, sim.y * 1000, 'Color', c_sim, 'LineWidth', 0.8);
    end
    plot(path_xy(:, 1), path_xy(:, 2), '--', 'Color', c_path, 'LineWidth', 1.2);
    for i = 1:4
        plot(wp(i, 1) * 1000 + 25 * cos(th), wp(i, 2) * 1000 + 25 * sin(th), 'k:');
    end
    plot(0, 0, 'k+');
    xlim([-160 160]); ylim([-160 160]);
    xlabel('x (mm)'); ylabel('y (mm)'); title(titles{a});
end
exportgraphics(f6, fullfile(out_dir, 'fig6_experiment_xy.png'), 'Resolution', 200);

% Figure 7: 20 s of the measured run
t0 = T.elapsed_s(1);
sel = T.elapsed_s <= t0 + 20;
f7 = light_figure([100 100 860 720]);
subplot(3, 1, 1); hold on; grid on;
plot(T.elapsed_s(sel) - t0, T.ball_x_m(sel) * 1000, 'Color', c_exp, 'LineWidth', 1.3);
plot(T.elapsed_s(sel) - t0, T.ball_y_m(sel) * 1000, 'Color', c_sim, 'LineWidth', 1.3);
stairs(T.elapsed_s(sel) - t0, T.target_x_m(sel) * 1000, ':', 'Color', c_exp);
stairs(T.elapsed_s(sel) - t0, T.target_y_m(sel) * 1000, ':', 'Color', c_sim);
ylabel('Position (mm)'); legend({'x', 'y'}, 'Location', 'eastoutside');
title('Measured run on the platform (dotted = active waypoint)');
subplot(3, 1, 2); hold on; grid on;
plot(T.elapsed_s(sel) - t0, T.speed_m_s(sel) * 1000, 'Color', c_path, 'LineWidth', 1.3);
ylabel('Speed (mm/s)'); legend({'speed'}, 'Location', 'eastoutside');
subplot(3, 1, 3); hold on; grid on;
plot(T.elapsed_s(sel) - t0, T.pitch_deg(sel), 'Color', c_exp, 'LineWidth', 1.1);
plot(T.elapsed_s(sel) - t0, T.roll_deg(sel), 'Color', c_sim, 'LineWidth', 1.1);
yline(5, '--', 'Color', c_path); yline(-5, '--', 'Color', c_path);
ylim([-5.8 5.8]);
ylabel('Tilt (deg)'); xlabel('Time (s)'); legend({'pitch', 'roll'}, 'Location', 'eastoutside');
exportgraphics(f7, fullfile(out_dir, 'fig7_experiment_time.png'), 'Resolution', 200);

fprintf('\nFigures saved in %s\n', out_dir);


function legs = experiment_legs(T)
%EXPERIMENT_LEGS  [time_s, overshoot_mm, arrival_speed_mm_s] per waypoint.
%
%   A leg ends when the active waypoint changes. Overshoot is the largest
%   distance past the waypoint, along the line from the previous waypoint,
%   from the start of the leg until 2 s after arrival (with touch and go the
%   ball is already being sent elsewhere when it arrives, so the overshoot
%   shows up just after the change).

t = T.elapsed_s;  x = T.ball_x_m;  y = T.ball_y_m;
tx = T.target_x_m;  ty = T.target_y_m;  sp = T.speed_m_s;

change = find((diff(tx) ~= 0 | diff(ty) ~= 0) & diff(t) < 0.5) + 1;

legs = zeros(0, 3);

for n = 2:numel(change)

    i = change(n);                  % first sample of the next leg
    j = change(n - 1);              % first sample of this leg

    W = [tx(i - 1), ty(i - 1)];     % waypoint just reached
    P = [tx(j - 1), ty(j - 1)];     % waypoint before it
    u = W - P;
    len = norm(u);

    if len < 0.01, continue; end
    u = u / len;

    last = find(t <= t(i) + 2.0, 1, 'last');
    if any(diff(t(j:last)) > 0.5), continue; end       % skip pauses

    past = (x(j:last) - W(1)) * u(1) + (y(j:last) - W(2)) * u(2);

    legs(end + 1, :) = [t(i) - t(j), max(0, max(past)) * 1000, sp(i) * 1000]; %#ok<AGROW>
end

end
