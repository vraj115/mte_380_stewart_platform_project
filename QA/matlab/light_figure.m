function f = light_figure(position)
%LIGHT_FIGURE  New figure with a white background, for use in a report.
%
%   Recent MATLAB releases follow the desktop's dark mode, which gives dark
%   figures. This asks for the light theme where that option exists.

f = figure('Color', 'w', 'Position', position);

try
    theme(f, 'light');          % R2025a and later
catch
    % Older releases are always light
end

end
