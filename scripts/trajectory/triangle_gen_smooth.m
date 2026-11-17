function [x, y, vx, vy, ax, ay] = triangle_gen_smooth(t)
% TRIANGLE_GEN_SMOOTH  Equilateral triangle for the tip C, for a MATLAB Function block.
%   1. t < t_app          : straight line from home to the first vertex (no step)
%   2. t < t_app + t_wait : stop at the first vertex
%   3. then               : laps through the 3 vertices, each side with a quintic
%                           time law (zero speed and acceleration at every vertex)
% The centroid of the triangle is at c. Joint limit +-20 deg: side <= 67 mm keeps
% both joints within 18 deg (60 mm -> 16 deg). Check with check_trajectory.m.

%% Settings (change here)
speed  = 1;                 % [-] 1 = one lap in T0; 2 = twice as fast (acceleration x4)
T0     = 4;                 % [s] lap time at speed = 1
s      = 0.060;             % [m] side of the triangle
c      = [0.127; 0.127];    % [m] centroid of the triangle (home of the tip)
home   = [0.127; 0.127];    % [m] tip position at t = 0
t_app  = 1.0;               % [s] time of the approach home -> first vertex
t_wait = 0.5;               % [s] stop at the first vertex before the laps
n_laps = Inf;               % number of laps, then stop at the first vertex
ccw    = true;              % true = counter-clockwise, false = clockwise

%% Vertices: bottom-left, bottom-right, top, back to bottom-left
h = s*sqrt(3)/2;            % height
W = c + [-s/2,  s/2,   0,     -s/2;
         -h/3, -h/3,   2*h/3, -h/3];
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