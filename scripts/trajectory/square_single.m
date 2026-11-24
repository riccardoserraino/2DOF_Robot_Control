function [x, y, vx, vy, ax, ay] = square_single(t)
% SQUARE_GEN_SMOOTH  Square path for the tip C, ONE speed per run.
% Self-contained: paste the whole file (local function included) into the MATLAB Function
% block of Model_pp_obs.slx and CNC_pp_obs.slx, and keep it in scripts/trajectory.
%
% PATH (t >= 0, the Simulink time)
%   1. t < t_app          : straight line from home to the first corner (no step)
%   2. t < t_app + t_wait : stop at the first corner
%   3. n_laps laps        : through the 4 corners, each side with a quintic time law
%                           (zero speed and acceleration at every corner)
%   4. then               : stop at the first corner
% Outputs: position [m], velocity [m/s], acceleration [m/s^2] of the tip.
%
% QUERY (t < 0, never used by Simulink, only by check_trajectory.m):
%   x = simulation length [s],  y = start of the laps [s],  vx = lap time [s],
%   vy = speed,  ax = end of the laps [s]

%% Settings (change here)
speed  = 1;                 % [-] 1 = one lap in T0; 2 = twice as fast (acceleration x4)
T0     = 4;                 % [s] lap time at speed = 1
n_laps = 2;                 % number of laps
s      = 0.07;             % [m] side of the square (<= 66 mm: joints within 18 deg)
c      = [0.127; 0.127];    % [m] centre of the square
home   = [0.127; 0.127];    % [m] tip position at t = 0
t_app  = 1.0;               % [s] approach home -> first corner
t_wait = 0.5;               % [s] stop at the first corner before the laps
t_tail = 0.5;               % [s] simulated time after the last lap
ccw    = true;              % true = counter-clockwise

%% Timing
T  = T0/speed;              % [s] lap time
Tk = T/4;                   % [s] time per side
t0 = t_app + t_wait;        % [s] start of the laps

%% Query (t < 0)
if t < 0
    x = t0 + n_laps*T + t_tail;  y = t0;  vx = T;  vy = speed;  ax = t0 + n_laps*T;  ay = 0;
    return
end

%% Corners: SW, SE, NE, NW, back to SW
h = s/2;
W = [c(1)-h, c(1)+h, c(1)+h, c(1)-h, c(1)-h;
     c(2)-h, c(2)-h, c(2)+h, c(2)+h, c(2)-h];
if ~ccw, W = fliplr(W); end

%% Path
tl = t - t0;                                % time since the start of the laps
if t < t_app
    [p, v, a] = quintic(home, W(:, 1), t, t_app);           % 1. approach
elseif tl < 0 || tl >= n_laps*T
    p = W(:, 1);  v = [0; 0];  a = [0; 0];                   % 2. and 4. stopped
else
    tl = mod(tl, T);                                         % 3. time inside the lap
    k  = min(floor(tl/Tk), 3) + 1;                           %    current side (1..4)
    [p, v, a] = quintic(W(:, k), W(:, k + 1), tl - (k - 1)*Tk, Tk);
end
x = p(1);  y = p(2);  vx = v(1);  vy = v(2);  ax = a(1);  ay = a(2);
end

%% ---------------- local functions ----------------
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
