function [x, y, vx, vy, ax, ay] = traj_sequence(t)
% TRAJ_SEQUENCE  Benchmark path: several shapes in sequence, each at several speeds.
% Self-contained (works in the MATLAB Function block through traj_select.m).
%
% PATH (t >= 0, the Simulink time)
%   1. home -> start point of the first shape in t_app (quintic: no step), wait t_wait
%   2. for each shape in "order":
%        laps_per_level laps at each speed of its row in "speeds" (t_pause between levels)
%        then move to the start point of the next shape in t_move (quintic: no step),
%        and wait t_wait
%   3. stop at the start point of the last shape
% Every side / lap starts and ends at rest, so changing speed or shape never makes a step.
% Outputs: position [m], velocity [m/s], acceleration [m/s^2] of the tip.
%
% SPEEDS: 1 = lap of T0 seconds. The other values come from check_trajectory.m run on the
% single file of each shape (square_single, circle_single, rhombus_single) at speed 1:
% it prints the speed for each target voltage. Default: 1, about 3 V, about 6 V
% (saturation) of feedforward, with the same sizes as the single files.
%
% QUERY (t < 0, never used by Simulink, only by check_trajectory.m)
%   t = -1     : x = Stop Time to set, y = start of the first lap, vx = lap time of
%                level 1, vy = speed of level 1, ax = end of the motion, ay = number of levels
%   t = -1 - i : level i: x = speed, y = start time [s], vx = end time [s], vy = shape code

%% Settings (change here)
order = [2, 1, 3];                 % shapes in sequence: 1 = square, 2 = circle, 3 = rhombus

% speed levels, one ROW per shape code (same number of levels on every row)
speeds = [1, 3.97, 6.64;           % 1 square : 4 s lap, ~3 V, ~6 V (saturation)
          1, 4.18, 6.91;           % 2 circle
          1, 5.23, 8.55];          % 3 rhombus
T0     = [4; 4; 4];                % [s] lap time at speed 1 of each shape (same rows)
laps_per_level = 2;                % laps at each speed

% geometry (keep it equal to the single files, so their speeds stay valid)
c     = [0.127; 0.127];            % [m] centre of every shape
home  = [0.127; 0.127];            % [m] tip position at t = 0
side  = 0.070;                     % [m] square side
D     = 0.085;                     % [m] circle diameter
th0   = 5*pi/4;                    % [rad] start point on the circle (bottom-left)
dx    = 0.085;                     % [m] rhombus horizontal diagonal
dy    = 0.060;                     % [m] rhombus vertical diagonal
ccw   = true;                      % true = counter-clockwise

% timing
t_app   = 1.0;                     % [s] home -> first shape
t_wait  = 0.5;                     % [s] stop at a shape's start point before its first level
t_pause = 0.5;                     % [s] stop between two levels of the same shape
t_move  = 1.0;                     % [s] move from one shape to the next
t_tail  = 0.5;                     % [s] still time after the last level

%% Geometry of the shapes
G.c = c;  G.r = D/2;  G.th0 = th0;
G.dirn = 1;                                                 % circle direction: +1 = ccw
if ~ccw, G.dirn = -1; end
h = side/2;
G.Wsq = [c(1)-h, c(1)+h, c(1)+h, c(1)-h, c(1)-h;            % square: SW, SE, NE, NW, SW
         c(2)-h, c(2)-h, c(2)+h, c(2)+h, c(2)-h];
hx = dx/2;  hy = dy/2;
G.Wrh = [c(1)-hx, c(1),    c(1)+hx, c(1),    c(1)-hx;      % rhombus: left, bottom,
         c(2),    c(2)-hy, c(2),    c(2)+hy, c(2)   ];     %          right, top, left
if ~ccw, G.Wsq = fliplr(G.Wsq);  G.Wrh = fliplr(G.Wrh); end

%% Timing of every level and of every move between shapes
nS = numel(order);  nL = size(speeds, 2);  nLev = nS*nL;
lev_shape = zeros(1, nLev);  lev_spd = zeros(1, nLev);  lev_T = zeros(1, nLev);
lev_t0    = zeros(1, nLev);  lev_dur = zeros(1, nLev);
mv_t0 = zeros(1, nS);  mv_T = zeros(1, nS);   % move TO shape j (j = 1: from home)
tt = 0;  m = 0;
for j = 1:nS
    if j == 1, mv_T(j) = t_app; else, mv_T(j) = t_move; end
    mv_t0(j) = tt;
    tt = tt + mv_T(j) + t_wait;
    for i = 1:nL
        m = m + 1;
        id = order(j);
        lev_shape(m) = id;
        lev_spd(m)   = speeds(id, i);
        lev_T(m)     = T0(id)/speeds(id, i);                % lap time of this level
        lev_t0(m)    = tt;
        lev_dur(m)   = laps_per_level*lev_T(m);
        tt = tt + lev_dur(m);
        if i < nL, tt = tt + t_pause; end
    end
end
t_mot = tt;                                                 % end of the motion

%% Query (t < 0)
if t < 0
    x = 0;  y = 0;  vx = 0;  vy = 0;  ax = 0;  ay = 0;
    i = round(-t) - 1;
    if i == 0
        x = t_mot + t_tail;  y = lev_t0(1);  vx = lev_T(1);  vy = lev_spd(1);  ax = t_mot;  ay = nLev;
    elseif i >= 1 && i <= nLev
        x = lev_spd(i);  y = lev_t0(i);  vx = lev_t0(i) + lev_dur(i);  vy = lev_shape(i);
    end
    return
end

%% Path
j_now = 1;                                                  % current shape: last move started
for j = 1:nS
    if t >= mv_t0(j), j_now = j; end
end
p = start_point(order(j_now), G);  v = [0; 0];  a = [0; 0]; % default: still at its start point
if t < mv_t0(j_now) + mv_T(j_now)                           % moving to this shape
    if j_now == 1, A = home; else, A = start_point(order(j_now - 1), G); end
    [p, v, a] = quintic(A, p, t - mv_t0(j_now), mv_T(j_now));
end
for m = 1:nLev                                              % inside a level? -> on the shape
    tl = t - lev_t0(m);
    if tl >= 0 && tl < lev_dur(m)
        [p, v, a] = shape_lap(lev_shape(m), mod(tl, lev_T(m)), lev_T(m), G);
    end
end
x = p(1);  y = p(2);  vx = v(1);  vy = v(2);  ax = a(1);  ay = a(2);
end

%% ---------------- local functions ----------------
function P = start_point(id, G)
% First point of shape id (where its laps start and end)
if id == 2
    P = G.c + G.r*[cos(G.th0); sin(G.th0)];
elseif id == 3
    P = G.Wrh(:, 1);
else
    P = G.Wsq(:, 1);
end
end

function [p, v, a] = shape_lap(id, tl, T, G)
% Point of shape id at time tl inside a lap of T seconds
if id == 2                                   % circle: the angle follows a quintic over the lap
    f     = tl/T;
    sig   = 10*f^3 - 15*f^4 + 6*f^5;
    dsig  = (30*f^2 - 60*f^3 + 30*f^4)/T;
    ddsig = (60*f - 180*f^2 + 120*f^3)/T^2;
    th   = G.th0 + G.dirn*2*pi*sig;
    dth  = G.dirn*2*pi*dsig;
    ddth = G.dirn*2*pi*ddsig;
    p = G.c + G.r*[cos(th); sin(th)];
    v = G.r*dth*[-sin(th); cos(th)];
    a = G.r*[-cos(th)*dth^2 - sin(th)*ddth;
             -sin(th)*dth^2 + cos(th)*ddth];
elseif id == 3
    [p, v, a] = polygon_lap(G.Wrh, tl, T);   % rhombus
else
    [p, v, a] = polygon_lap(G.Wsq, tl, T);   % square
end
end

function [p, v, a] = polygon_lap(W, tl, T)
% 4 sides, each a quintic in T/4 seconds
Tk = T/4;
k  = min(floor(tl/Tk), 3) + 1;
[p, v, a] = quintic(W(:, k), W(:, k + 1), tl - (k - 1)*Tk, Tk);
end

function [p, v, a] = quintic(A, B, tk, Tk)
% Straight line A -> B in Tk seconds with zero speed and acceleration at both ends:
% s(f) = 10f^3 - 15f^4 + 6f^5, f = tk/Tk.
f     = min(max(tk/Tk, 0), 1);
sig   = 10*f^3 - 15*f^4 + 6*f^5;
dsig  = (30*f^2 - 60*f^3 + 30*f^4)/Tk;
ddsig = (60*f - 180*f^2 + 120*f^3)/Tk^2;
d = B - A;
p = A + sig*d;  v = dsig*d;  a = ddsig*d;
end
