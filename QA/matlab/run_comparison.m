%RUN_COMPARISON  Figures 1-4 of the report: conventional position control
%   against trajectory-planned control on the diamond.
%
%   Run from this folder:   run_comparison
%
%   Two plants are simulated, one after the other:
%     'ideal'     the model derived in the report, x'' = (3/5) g theta,
%                 with no delay
%     'measured'  the plant identified from the platform logs:
%                 0.092 m/s^2 per degree with 0.23 s of dead time
%
%   Both strategies use the same gains, limits, waypoints and start point.
%   Figures are saved as PNG files in ../figures.

clear; close all; clc;

PLANTS = {'ideal', 'measured'};
SIZE   = 0.08;                   % diamond centre-to-corner distance, m

for n = 1:numel(PLANTS)

PLANT = PLANTS{n};

p  = default_params(PLANT);
wp = diamond_waypoints(SIZE);

pos = simulate_ball('position', wp, p);
pln = simulate_ball('planned',  wp, p);

% ----- Results table -----
fprintf('\nPlant: %s  (Kb = %.4f m/s^2 per deg, dead time = %.2f s)\n', ...
    PLANT, p.Kb_true, p.delay);
fprintf('Gains: kp = %g, ki = %g, kd = %g\n', p.kp, p.ki, p.kd);
fprintf('Planner: go tilt = %g deg, brake = %g m/s^2, cruise = %g m/s\n\n', ...
    p.go_tilt, p.brake_accel, p.cruise);

print_legs('Conventional position control', pos);
print_legs('Trajectory-planned control',    pln);

% ----- Colours -----
c_pos  = [0.82 0.29 0.36];
c_pln  = [0.04 0.43 0.60];
c_path = [0.35 0.35 0.35];

out_dir = fullfile('..', 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
tag = PLANT;

path_xy = [p.start; wp] * 1000;

% ===== Figure 1: X-Y trajectory =====
f1 = light_figure([100 100 640 600]);
hold on; grid on; axis equal;
plot(path_xy(:, 1), path_xy(:, 2), '--', 'Color', c_path, 'LineWidth', 1.2);
plot(pos.x * 1000, pos.y * 1000, 'Color', c_pos, 'LineWidth', 1.6);
plot(pln.x * 1000, pln.y * 1000, 'Color', c_pln, 'LineWidth', 1.6);
th = linspace(0, 2 * pi, 60);
for i = 1:size(wp, 1)
    plot(wp(i, 1) * 1000 + p.tolerance * 1000 * cos(th), ...
         wp(i, 2) * 1000 + p.tolerance * 1000 * sin(th), ':', 'Color', c_path);
end
plot(path_xy(:, 1), path_xy(:, 2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
xlabel('x (mm)'); ylabel('y (mm)');
title(sprintf('Ball path on the diamond (%s plant)', PLANT));
legend({'Desired path', 'Position control', 'Trajectory planned', 'Target circle'}, ...
    'Location', 'northeastoutside');
exportgraphics(f1, fullfile(out_dir, ['fig1_xy_path_' tag '.png']), 'Resolution', 200);

% ===== Figure 2: X and Y position against time =====
f2 = light_figure([100 100 820 560]);
names = {'x', 'y'};
for a = 1:2
    subplot(2, 1, a); hold on; grid on;
    if a == 1, yp = pos.x; yl = pln.x; else, yp = pos.y; yl = pln.y; end
    plot(pos.t, yp * 1000, 'Color', c_pos, 'LineWidth', 1.5);
    plot(pln.t, yl * 1000, 'Color', c_pln, 'LineWidth', 1.5);
    stairs(pos.t, pos.target(:, a) * 1000, ':', 'Color', c_pos);
    stairs(pln.t, pln.target(:, a) * 1000, ':', 'Color', c_pln);
    ylabel(sprintf('%s (mm)', names{a}));
    if a == 1
        title(sprintf('Ball position, dotted = active waypoint (%s plant)', PLANT));
        legend({'Position control', 'Trajectory planned'}, 'Location', 'best');
    end
end
xlabel('Time (s)');
exportgraphics(f2, fullfile(out_dir, ['fig2_position_' tag '.png']), 'Resolution', 200);

% ===== Figure 3: speed against time =====
f3 = light_figure([100 100 820 380]);
hold on; grid on;
plot(pos.t, pos.speed * 1000, 'Color', c_pos, 'LineWidth', 1.5);
plot(pln.t, pln.speed * 1000, 'Color', c_pln, 'LineWidth', 1.5);
yline(p.cruise * 1000, '--', 'Color', c_path);
xlabel('Time (s)'); ylabel('Ball speed (mm/s)');
title(sprintf('Ball speed (%s plant)', PLANT));
legend({'Position control', 'Trajectory planned', 'Planner cruise speed'}, 'Location', 'best');
exportgraphics(f3, fullfile(out_dir, ['fig3_speed_' tag '.png']), 'Resolution', 200);

% ===== Figure 4: commanded roll and pitch =====
f4 = light_figure([100 100 820 560]);
labels = {'Pitch (deg)', 'Roll (deg)'};
for a = 1:2
    subplot(2, 1, a); hold on; grid on;
    if a == 1, yp = pos.pitch; yl = pln.pitch; else, yp = pos.roll; yl = pln.roll; end
    plot(pos.t, yp, 'Color', c_pos, 'LineWidth', 1.3);
    plot(pln.t, yl, 'Color', c_pln, 'LineWidth', 1.3);
    yline( p.max_tilt, '--', 'Color', c_path);
    yline(-p.max_tilt, '--', 'Color', c_path);
    ylim([-1.15 1.15] * p.max_tilt);
    ylabel(labels{a});
    if a == 1
        title(sprintf('Commanded platform tilt, dashed = limit (%s plant)', PLANT));
        legend({'Position control', 'Trajectory planned'}, 'Location', 'best');
    end
end
xlabel('Time (s)');
exportgraphics(f4, fullfile(out_dir, ['fig4_tilt_' tag '.png']), 'Resolution', 200);

fprintf('Figures saved in %s\n', out_dir);

end


function print_legs(name, r)
%PRINT_LEGS  One line per waypoint, then a summary line.

fprintf('%s\n', name);
fprintf('  waypoint   overshoot (mm)   time (s)   peak speed (mm/s)\n');
for i = 1:size(r.legs, 1)
    note = '';
    if r.legs(i, 4), note = '  (timed out)'; end
    fprintf('  %5d %14.1f %12.2f %14.0f%s\n', r.legs(i, 1), ...
        r.legs(i, 2) * 1000, r.legs(i, 3), r.legs(i, 6) * 1000, note);
end
fprintf('  mean overshoot %.1f mm, worst %.1f mm, lap time %.1f s, peak tilt %.2f deg\n\n', ...
    mean(r.legs(:, 2)) * 1000, max(r.legs(:, 2)) * 1000, r.t(end), ...
    max(hypot(r.pitch, r.roll)));

end
