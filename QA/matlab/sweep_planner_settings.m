%SWEEP_PLANNER_SETTINGS  Effect of the planned acceleration and braking
%   magnitudes on overshoot and travel time (Figure 5 of the report).
%
%   Run from this folder:   sweep_planner_settings
%
%   For each combination of accelerating tilt and braking deceleration the
%   diamond is simulated with the trajectory planner. Mean overshoot past
%   the waypoints and the lap time are plotted, for the ideal model and for
%   the plant measured on the platform.

clear; close all; clc;

SIZE = 0.08;
brakes   = [0.04 0.06 0.08 0.10 0.12 0.16 0.20 0.25 0.30];   % m/s^2
go_tilts = [2 3 4];                                          % deg
plants   = {'ideal', 'measured'};

wp = diamond_waypoints(SIZE);

over = zeros(numel(plants), numel(go_tilts), numel(brakes));
lap  = over;

for a = 1:numel(plants)
    for b = 1:numel(go_tilts)
        for c = 1:numel(brakes)

            p = default_params(plants{a});
            p.go_tilt     = go_tilts(b);
            p.brake_accel = brakes(c);

            r = simulate_ball('planned', wp, p);

            over(a, b, c) = mean(r.legs(:, 2)) * 1000;
            lap(a, b, c)  = r.t(end);
        end
    end
end

% Reference: position control with the same gains
ref = zeros(numel(plants), 2);
for a = 1:numel(plants)
    r = simulate_ball('position', wp, default_params(plants{a}));
    ref(a, :) = [mean(r.legs(:, 2)) * 1000, r.t(end)];
end

% ----- Table -----
for a = 1:numel(plants)
    fprintf('\nPlant: %s   (position control: overshoot %.1f mm, lap %.1f s)\n', ...
        plants{a}, ref(a, 1), ref(a, 2));
    fprintf('  go tilt   brake    brake tilt   mean overshoot   lap time\n');
    fprintf('   (deg)   (m/s^2)     (deg)          (mm)           (s)\n');
    for b = 1:numel(go_tilts)
        for c = 1:numel(brakes)
            fprintf('  %5.1f %9.2f %10.2f %14.1f %14.1f\n', go_tilts(b), brakes(c), ...
                brakes(c) / 0.10273, over(a, b, c), lap(a, b, c));
        end
    end
end

% ----- Figure 5 -----
colours = [0.04 0.43 0.60; 0.91 0.47 0.10; 0.11 0.54 0.24];

f = light_figure([100 100 900 620]);
for a = 1:numel(plants)

    subplot(2, 2, a); hold on; grid on;
    for b = 1:numel(go_tilts)
        plot(brakes, squeeze(over(a, b, :)), '-o', 'Color', colours(b, :), ...
            'LineWidth', 1.4, 'MarkerFaceColor', colours(b, :), 'MarkerSize', 4);
    end
    yline(ref(a, 1), '--', 'Color', [0.82 0.29 0.36]);
    title(sprintf('Overshoot (%s plant)', plants{a}));
    ylabel('Mean overshoot (mm)');
    if a == 1
        legend([arrayfun(@(g) sprintf('Accelerate at %g deg', g), go_tilts, ...
            'UniformOutput', false), {'Position control'}], 'Location', 'best');
    end

    subplot(2, 2, a + 2); hold on; grid on;
    for b = 1:numel(go_tilts)
        plot(brakes, squeeze(lap(a, b, :)), '-o', 'Color', colours(b, :), ...
            'LineWidth', 1.4, 'MarkerFaceColor', colours(b, :), 'MarkerSize', 4);
    end
    yline(ref(a, 2), '--', 'Color', [0.82 0.29 0.36]);
    title(sprintf('Lap time (%s plant)', plants{a}));
    xlabel('Planned braking deceleration (m/s^2)');
    ylabel('Lap time (s)');
end

out_dir = fullfile('..', 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
exportgraphics(f, fullfile(out_dir, 'fig5_planner_sweep.png'), 'Resolution', 200);
fprintf('\nFigure saved in %s\n', out_dir);
