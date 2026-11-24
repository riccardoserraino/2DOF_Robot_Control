function [x, y, vx, vy, ax, ay] = square_speeds(t)
% SQUARE_SPEEDS_GEN  Square path with SEVERAL SPEED LEVELS in one run: tests the
% controller from low to high voltage. Self-contained: paste the whole file (local
% function included) into a MATLAB Function block, and keep it in scripts/trajectory.
%
% PATH (t >= 0, the Simulink time)
%   1. approach : straight line from home to the first corner in t_app (no step)
%   2. wait     : stop at the first corner for t_wait
%   3. levels   : laps_per_level laps at each value of "speeds", slowest first, with a
%                 stop of t_pause at the corner between two levels (the error settles,
%                 so every level starts from rest and can be analysed on its own)
%   4. stop     : at the first corner after the last level
% Every side is a quintic (zero speed and acceleration at the corners): no steps.
% Outputs: position [m], velocity [m/s], acceleration [m/s^2] of the tip.
%
% SPEEDS: run check_trajectory.m with traj = @square_gen_smooth (speed = 1); it prints
% the speeds that give 2, 3, 4, 5, 6 V of feedforward. Paste them below.
%
% QUERY (t < 0, never used by Simulink, only by check_trajectory.m)
%   t = -1     : x = Stop Time to set, y = start of the first lap, vx = lap time of
%                level 1, vy = speed of level 1, ax = end of the motion, ay = number of levels
%   t = -1 - i : level i:  x = speed, y = start time [s], vx = end time [s]

%% Settings (change here)
speeds         = [1 2.79 3.97 4.97 5.85 6.64];   % [-] about 1, 2, 3, 4, 5, 6 V of feedforward
laps_per_level = 2;                 % laps at each speed
T0      = 4;                        % [s] lap time at speed = 1
side    = 0.070;                    % [m] side of the square (<= 66 mm: joints within 18 deg)
c       = [0.127; 0.127];           % [m] centre of the square
home    = [0.127; 0.127];           % [m] tip position at t = 0
t_app   = 1.0;                      % [s] approach home -> first corner
t_wait  = 0.5;                      % [s] stop at the first corner before the first level
t_pause = 0.5;                      % [s] stop at the corner between two levels
t_tail  = 0.5;                      % [s] still time after the last level
ccw     = true;                     % true = counter-clockwise

%% Timing of the levels
nLev  = numel(speeds);
T     = T0./speeds;                                 % [s] lap time of each level
dur   = laps_per_level*T;                           % [s] duration of each level
t_lev = t_app + t_wait + [0, cumsum(dur(1:nLev-1) + t_pause)];   % [s] start of each level
t_mot = t_lev(nLev) + dur(nLev);                    % [s] end of the motion

%% Query (t < 0)
if t < 0
    x = 0;  y = 0;  vx = 0;  vy = 0;  ax = 0;  ay = 0;
    i = round(-t) - 1;
    if i == 0
        x = t_mot + t_tail;  y = t_lev(1);  vx = T(1);  vy = speeds(1);  ax = t_mot;  ay = nLev;
    elseif i >= 1 && i <= nLev
        x = speeds(i);  y = t_lev(i);  vx = t_lev(i) + dur(i);
    end
    return
end

%% Corners: SW, SE, NE, NW, back to SW
h = side/2;
W = [c(1)-h, c(1)+h, c(1)+h, c(1)-h, c(1)-h;
     c(2)-h, c(2)-h, c(2)+h, c(2)+h, c(2)-h];
if ~ccw, W = fliplr(W); end

%% Path
p = W(:, 1);  v = [0; 0];  a = [0; 0];              % default: stopped at the first corner
if t < t_app
    [p, v, a] = quintic(home, W(:, 1), t, t_app);   % 1. approach
else
    for i = 1:nLev                                  % 3. inside a level? (else: stopped)
        tl = t - t_lev(i);                          %    time since the start of level i
        if tl >= 0 && tl < dur(i)
            tl = mod(tl, T(i));                     %    time inside the lap
            Tk = T(i)/4;                            %    time per side
            k  = min(floor(tl/Tk), 3) + 1;          %    current side (1..4)
            [p, v, a] = quintic(W(:, k), W(:, k + 1), tl - (k - 1)*Tk, Tk);
        end
    end
end
x = p(1);  y = p(2);  vx = v(1);  vy = v(2);  ax = a(1);  ay = a(2);
end

%% ---------------- local functions ----------------
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
