function [x, y, vx, vy, ax, ay] = star_gen_smooth(t)
% STAR_GEN_SMOOTH  Five-pointed star for the tip C, for a MATLAB Function block.
%   10 vertices alternating between the outer radius R (tips) and the inner radius
%   ratio*R (notches); ratio = 0.382 gives the regular star (pentagram outline).
%   1. t < t_app          : straight line from home to the top tip (no step)
%   2. t < t_app + t_wait : stop at the top tip
%   3. then               : laps through the 10 vertices, each segment with a quintic
%                           time law (zero speed and acceleration at every vertex)
% The hardest of the four paths: 10 starts and stops per lap, so with the same lap
% time it needs about 3 times the acceleration of the square.
% Joint limit +-20 deg: outer diameter <= 78 mm keeps both joints within 18 deg
% (70 mm -> 16 deg). Check with check_trajectory.m.

%% Settings (change here)
speed  = 1;                 % [-] 1 = one lap in T0; 2 = twice as fast (acceleration x4)
T0     = 6;                 % [s] lap time at speed = 1 (10 segments: slower than the square)
D      = 0.070;             % [m] outer diameter (circle through the 5 tips)
ratio  = 0.382;             % [-] inner/outer radius (0.382 = regular star)
c      = [0.127; 0.127];    % [m] centre of the star (home of the tip)
home   = [0.127; 0.127];    % [m] tip position at t = 0
t_app  = 1.0;               % [s] time of the approach home -> top tip
t_wait = 0.5;               % [s] stop at the top tip before the laps
n_laps = Inf;               % number of laps, then stop at the top tip
ccw    = true;              % true = counter-clockwise, false = clockwise

%% Vertices: top tip, then alternating notch / tip every 36 deg, back to the top tip
R   = D/2;
ang = pi/2 + (0:10)*pi/5;                       % 11 angles (the last = the first)
rad = R*ones(1, 11);
rad(2:2:10) = ratio*R;                          % notches at the even positions
W = c + [rad.*cos(ang);
         rad.*sin(ang)];
if ~ccw, W = fliplr(W); end

[x, y, vx, vy, ax, ay] = follow_polygon(t, W, home, T0/speed, t_app, t_wait, n_laps);
end

%% ---------------- local functions ----------------
function [x, y, vx, vy, ax, ay] = follow_polygon(t, W, home, T, t_app, t_wait, n_laps)
% Approach, stop, then laps through the waypoints W (first = last), same time per side.
n  = size(W, 2) - 1;                        % number of segments
Tk = T/n;                                   % time per segment
tl = t - t_app - t_wait;                    % time since the start of the laps
if t < t_app
    [p, v, a] = quintic(home, W(:, 1), t, t_app);
elseif tl < 0 || tl >= n_laps*T
    p = W(:, 1);  v = [0; 0];  a = [0; 0];  % stopped at the first vertex
else
    tl = mod(tl, T);                        % time inside the lap
    k  = min(floor(tl/Tk), n - 1) + 1;      % current segment (1..n)
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