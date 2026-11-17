function [x, y, vx, vy, ax, ay] = circle_gen_smooth(t)
% CIRCLE_GEN_SMOOTH  Circle for the tip C, for a MATLAB Function block.
%   1. t < t_app          : straight line from home to the start point (no step)
%   2. t < t_app + t_wait : stop at the start point
%   3. then               : laps; the angle follows a quintic time law over each lap
%                           (zero speed and acceleration at the start of every lap)
% Joint limit +-20 deg: diameter <= 78 mm keeps both joints within 18 deg
% (70 mm -> 16 deg). Check with check_trajectory.m.

%% Settings (change here)
speed  = 1;                 % [-] 1 = one lap in T0; 2 = twice as fast (acceleration x4)
T0     = 4;                 % [s] lap time at speed = 1
D      = 0.070;             % [m] diameter
c      = [0.127; 0.127];    % [m] centre of the circle (home of the tip)
home   = [0.127; 0.127];    % [m] tip position at t = 0
th0    = 5*pi/4;            % [rad] start point on the circle (5*pi/4 = bottom-left)
t_app  = 1.0;               % [s] time of the approach home -> start point
t_wait = 0.5;               % [s] stop at the start point before the laps
n_laps = Inf;               % number of laps, then stop at the start point
ccw    = true;              % true = counter-clockwise, false = clockwise

%% Approach, stop, laps
T  = T0/speed;
r  = D/2;
P0 = c + r*[cos(th0); sin(th0)];            % start point on the circle
tl = t - t_app - t_wait;                    % time since the start of the laps
if t < t_app
    [p, v, a] = quintic(home, P0, t, t_app);
elseif tl < 0 || tl >= n_laps*T
    p = P0;  v = [0; 0];  a = [0; 0];
else
    f     = mod(tl, T)/T;                   % 0 -> 1 inside the lap
    sig   = 10*f^3 - 15*f^4 + 6*f^5;
    dsig  = (30*f^2 - 60*f^3 + 30*f^4)/T;
    ddsig = (60*f - 180*f^2 + 120*f^3)/T^2;
    if ccw, dirn = 1; else, dirn = -1; end
    th   = th0 + dirn*2*pi*sig;             % angle [rad]
    dth  = dirn*2*pi*dsig;                  % angular speed [rad/s]
    ddth = dirn*2*pi*ddsig;                 % angular acceleration [rad/s^2]
    p = c + r*[cos(th); sin(th)];
    v = r*dth*[-sin(th); cos(th)];
    a = r*[-cos(th)*dth^2 - sin(th)*ddth;   % centripetal + tangential
           -sin(th)*dth^2 + cos(th)*ddth];
end
x = p(1);  y = p(2);  vx = v(1);  vy = v(2);  ax = a(1);  ay = a(2);
end

%% ---------------- local function ----------------
function [p, v, a] = quintic(A, B, tk, Tk)
% Straight line A -> B in Tk seconds with zero speed and acceleration at both ends.
f     = min(max(tk/Tk, 0), 1);
sig   = 10*f^3 - 15*f^4 + 6*f^5;
dsig  = (30*f^2 - 60*f^3 + 30*f^4)/Tk;
ddsig = (60*f - 180*f^2 + 120*f^3)/Tk^2;
d = B - A;
p = A + sig*d;  v = dsig*d;  a = ddsig*d;
end