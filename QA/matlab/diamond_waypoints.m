function wp = diamond_waypoints(size_m)
%DIAMOND_WAYPOINTS  Waypoints for one lap of the diamond, in metres.
%
%   The ball starts at the centre, visits the four corners, closes the
%   diamond at the first corner and returns to the centre:
%       (R,0) -> (0,R) -> (-R,0) -> (0,-R) -> (R,0) -> (0,0)

if nargin < 1
    size_m = 0.08;              % centre-to-corner distance used on the platform
end

R = size_m;

wp = [ R  0
       0  R
      -R  0
       0 -R
       R  0
       0  0 ];

end
