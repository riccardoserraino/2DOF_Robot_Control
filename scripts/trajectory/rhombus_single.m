function [x, y, vx, vy, ax, ay] = rhombus_single(t)
% RHOMBUS_SINGLE  Rhombus for the tip C, for a MATLAB Function block.
%   1. t < t_app          : straight line from home to the left vertex (no step)
%   2. t < t_app + t_wait : stop at the left vertex
%   3. then               : laps through the 4 vertices, each side with a quintic
%                           time law (zero speed and acceleration at every vertex)
% Vertices on the two axes through c: left, bottom, right, top. Every side moves BOTH
% joints together (the square sides move mostly one joint at a time): it tests the
% coupling of the linkage. Joint limit +-20 deg: 85 x 60 mm keeps both joints within
% 19.7 deg. Check with check_trajectory.m.

%% Settings (change here)
speed  = 1;                 % [-] 1 = one lap in T0; 2 = twice as fast (acceleration x4)
T0     = 4;                 % [s] lap time at speed = 1
dx     = 0.085;             % [m] horizontal diagonal (left - right vertex)
dy     = 0.050;             % [m] vertical diagonal (bottom - top vertex)
c      = [0.127; 0.127];    % [m] centre of the rhombus (home of the tip)
home   = [0.127; 0.127];    % [m] tip position at t = 0
t_app  = 1.0;               % [s] time of the approach home -> left vertex
t_wait = 0.5;               % [s] stop at the left vertex before the laps
n_laps = 2;                 % number of laps, then stop at the left vertex
t_tail = 0.5;               % [s] still time after the last lap
ccw    = true;              % true = counter-clockwise, false = clockwise

%% Query (t < 0, never used by Simulink, only by check_trajectory.m / traj_select(-1, id)):
%   x = Stop Time to set, y = start of the laps, vx = lap time, vy = speed, ax = end of the motion
T  = T0/speed;                              % [s] lap time
t0 = t_app + t_wait;                        % [s] start of the laps
if t < 0
    x = t0 + n_laps*T + t_tail;  y = t0;  vx = T;  vy = speed;  ax = t0 + n_laps*T;  ay = 0;
    return
end

%% Vertices: left, bottom, right, top, back to left
hx = dx/2;  hy = dy/2;      % half diagonals
W = c + [-hx,  0,    hx,  0,   -hx;
          0,  -hy,   0,   hy,   0 ];
if ~ccw, W = fliplr(W); end

[x, y, vx, vy, ax, ay] = follow_polygon(t, W, home, T0/speed, t_app, t_wait, n_laps);
end

%% ---------------- local functions ----------------
function [x, y, vx, vy, ax, ay] = follow_polygon(t, W, home, T, t_app, t_wait, n_laps)
% Approach, stop, then laps through the waypoints W (first = last), same time per side.
n  = size(W, 2) - 1;                        % number of sides
Tk = T/n;                                   % time per side
tl = t - t_app - t_wait;                    % time since the start of the laps
if t < t_app
    [p, v, a] = quintic(home, W(:, 1), t, t_app);
elseif tl < 0 || tl >= n_laps*T
    p = W(:, 1);  v = [0; 0];  a = [0; 0];  % stopped at the first vertex
else
    tl = mod(tl, T);                        % time inside the lap
    k  = min(floor(tl/Tk), n - 1) + 1;      % current side (1..n)
    [p, v, a] = quintic(W(:, k), W(:, k + 1), tl - (k - 1)*Tk, Tk);
end
x = p(1);  y = p(2);  vx = v(1);  vy = v(2);  ax = a(1);  ay = a(2);
end

function [p, v, a] = quintic(A, B, tk, Tk)
% Straight line A -> B in Tk seconds with zero speed and acceleration at both ends:
% s(f) = 10f^3 - 15f^4 + 6f^5, f = tk/Tk. Peak speed 1.875*|B-A|/Tk,
% peak acceleration 5.77*|B-A|/Tk^2.
f     = min(max(tk/Tk, 0), 1);
sig   = 10*f^3 - 15*f^4 + 6*f^5;
dsig  = (30*f^2 - 60*f^3 + 30*f^4)/Tk;
ddsig = (60*f - 180*f^2 + 120*f^3)/Tk^2;
d = B - A;
p = A + sig*d;  v = dsig*d;  a = ddsig*d;
end