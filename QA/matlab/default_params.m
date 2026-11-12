function p = default_params(plant)
%DEFAULT_PARAMS  Parameters for the ball-on-plate simulation.
%
%   p = default_params('ideal')     the model derived in the report:
%                                   x'' = (3/5) g theta, no delay
%   p = default_params('measured')  the plant identified from the logged run
%                                   waypoint_run_20261112_163556.csv:
%                                   0.092 m/s^2 per degree, 0.23 s dead time
%
%   Angles are in degrees, lengths in metres, times in seconds.

if nargin < 1
    plant = 'ideal';
end

g = 9.81;

% ----- Ball model used by the planner (the report's model) -----
p.Kb = (3/5) * g * pi / 180;    % 0.10273 m/s^2 per degree of tilt

% ----- The simulated plant -----
switch lower(plant)
    case 'ideal'
        p.Kb_true = p.Kb;       % plant equals the model
        p.delay   = 0.0;        % no dead time
    case 'measured'
        p.Kb_true = 0.092;      % identified from the logged run (R^2 = 0.85)
        p.delay   = 0.23;       % camera + serial + servo dead time
    otherwise
        error('Unknown plant "%s". Use ''ideal'' or ''measured''.', plant);
end

p.plant = lower(plant);

% ----- Feedback gains (the values that worked on the platform) -----
p.kp = 27.0;                    % deg per m
p.ki = 2.0;                     % deg per (m s)
p.kd = 20.5;                    % deg per (m/s)
p.d_filter = 0.08;              % derivative low-pass time constant, s
p.i_limit  = 2.5;               % largest tilt the integral term may ask for, deg

% ----- Platform limits -----
p.max_tilt  = 5.0;              % deg, combined roll/pitch
p.tilt_rate = 60.0;             % deg/s slew limit

% ----- Trajectory planner -----
p.go_tilt     = 3.0;            % deg, tilt used to accelerate
p.brake_accel = 0.12;           % m/s^2, planned braking deceleration
p.cruise      = 0.08;           % m/s, top speed between waypoints
p.model_delay = p.delay;        % dead time the planner compensates for
p.stuck_wait  = 0.35;           % s at rest before a new move is planned

% ----- Waypoint logic -----
p.tolerance   = 0.025;          % m, radius of the target circle
p.dwell       = 0.25;           % s the ball must rest in the circle
                                % (0 = touch and go)
p.leg_timeout = 30.0;           % s before a waypoint is skipped

% ----- Simulation -----
p.dt    = 1/30;                 % control period (camera frame rate), s
p.t_max = 120.0;                % s
p.start = [0 0];                % ball starts at rest at the centre, m
p.laps  = 1;

end
