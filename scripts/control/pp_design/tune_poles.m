%% TUNE_POLES - tune wn, zeta, alpha with sliders (design A pattern) for BOTH motors
% Poles (the same for motor 3165 and 3166, the gains differ because k and tau differ):
%   pair      -zeta*wn +- j*wn*sqrt(1 - zeta^2)    wn   = speed / stiffness of the joint
%                                                  zeta = damping of the pair
%   real pole -alpha*zeta*wn = -lambda              alpha = speed of the integral action
% "lock alpha" keeps lambda = lam_fix fixed: alpha = lam_fix/(zeta*wn), so moving wn or
% zeta never changes how fast friction and load errors are removed.
%
% Two windows:
%   PLOTS : pole map, 10 deg (req.R) step, voltage, load step, Bode of the loop.
%           Blue = motor 3165, orange = motor 3166, grey = PID of pid_tune.m.
%           Solid = beta = 1 (trajectories), dashed = beta* (steps).
%   STATS : sliders and boxes for wn, zeta, alpha, lock of alpha, gains and beta* of both
%           motors, requirement table of both motors (PID, beta*, beta = 1; '*' = not met),
%           "Write to Simulink" (same variables as design_A / check_design) and "Print".
% While a slider moves a FAST model is used (Pade delay); on release everything is EXACT.
clear; clc; close all
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% Settings
wn0     = 50;              % [rad/s] start value of wn
zeta0   = 0.9;             % [-]     start value of zeta
lam_fix = 3;               % [rad/s] integral pole used when alpha is locked
lock0   = true;            % true: alpha = lam_fix/(zeta*wn)
fps     = 15;              % max plot refreshes per second while a slider moves

% slider ranges [min max]
rng.wn    = [10 120];
rng.zeta  = [0.5 0.999];
rng.alpha = [0.005 1];

%% Models, PID reference (computed once)
[mot, req] = pp_setup();
S.mot  = mot;
S.req  = req;
S.lam  = lam_fix;
S.rng  = rng;
S.lock = lock0;
S.x    = [wn0, zeta0, lam_fix/(zeta0*wn0)];       % [wn zeta alpha]
if ~lock0, S.x(3) = 0.3; end
S.dirty = false;
S.tf  = linspace(0, req.t(end), 750)';           % coarse time grid (fast model)
S.w   = logspace(0, log10(pi/req.Ts), 200);      % Bode frequencies
S.wnn = logspace(2, log10(pi/req.Ts), 50);       % chatter frequencies
for i = 1:2
    S.pid{i} = closed_loop(mot(i), req, mot(i).Cpid);         % exact, once
    Gd = mot(i).G;   Gd.InputDelay = req.Td;
    S.Gp{i} = ss(pade(Gd, 2));                                % fast model of the delay
end

%% Windows
scr  = get(0, 'ScreenSize');
fig  = figure('Name', 'PLOTS - both motors', 'Units', 'normalized', ...
              'Position', [0.01 0.05 0.64 0.87], 'Color', 'w');
fig2 = figure('Name', 'STATS - wn, zeta, alpha', 'Units', 'pixels', ...
              'Position', [scr(3) - 720, 60, 700, 640], 'Color', 'w', ...
              'MenuBar', 'none', 'ToolBar', 'none');
S.fig2 = fig2;
S.h = init_plots(fig, S);
S.s = init_stats(fig2, fig, S);
set(fig,  'CloseRequestFcn', @(src, ~) on_close(fig, src));
set(fig2, 'CloseRequestFcn', @(src, ~) on_close(fig, src));
S.tmr = timer('ExecutionMode', 'fixedSpacing', 'Period', round(1000/fps)/1000, ...
              'BusyMode', 'drop', 'TimerFcn', @(~, ~) on_tick(fig));
guidata(fig, S);
sync_controls(S);
update(fig, 'exact');
start(S.tmr);

%% ========================= callbacks ====================================
function on_slide(fig, k, final)
% Slider k (1 wn, 2 zeta, 3 alpha) moved (final = false) or released (final = true)
S = guidata(fig);
S.x(k) = S.s.sl(k).Value;
S = apply_lock(S);
sync_controls(S);
if final
    S.dirty = false;   guidata(fig, S);   update(fig, 'exact');
else
    S.dirty = true;    guidata(fig, S);              % the timer refreshes (fast model)
end
end

function on_edit(fig, k)
% Value typed in box k
S = guidata(fig);
v = str2double(S.s.ed(k).String);
if ~isfinite(v), sync_controls(S); return; end
names = {'wn', 'zeta', 'alpha'};
r = S.rng.(names{k});
S.x(k) = min(max(v, r(1)), r(2));
S = apply_lock(S);
sync_controls(S);
S.dirty = false;   guidata(fig, S);   update(fig, 'exact');
end

function on_lock(fig)
S = guidata(fig);
S.lock = logical(S.s.lock.Value);
S = apply_lock(S);
sync_controls(S);
S.dirty = false;   guidata(fig, S);   update(fig, 'exact');
end

function S = apply_lock(S)
% Locked: the real pole stays at lam_fix whatever wn and zeta are
if S.lock
    S.x(3) = S.lam/(S.x(2)*S.x(1));
end
end

function on_tick(fig)
% Timer: refresh with the fast model if a slider moved
if ~isvalid(fig) || strcmp(fig.BeingDeleted, 'on'), return; end
S = guidata(fig);
if ~S.dirty, return; end
S.dirty = false;   guidata(fig, S);
update(fig, 'fast');
end

function on_write(fig)
% Same Simulink variables as design_A / check_design, for both motors
S = guidata(fig);
modes = {'one', 'star'};
pp_simulink(S.mot, S.req, poles_A(S.x), modes{S.s.beta.Value});
end

function on_print(fig)
S = guidata(fig);   p = poles_A(S.x);
fprintf('\nwn = %.2f rad/s   zeta = %.3f   alpha = %.4f   (real pole %.2f rad/s)\n', ...
        S.x(1), S.x(2), S.x(3), S.x(3)*S.x(2)*S.x(1));
fprintf('poles = %s rad/s\n', mat2str(p.', 4));
for i = 1:2
    g = pp_gains(S.mot(i), p);
    fprintf('motor %s: Kx = %s   Ki = %.4f   beta* = %.3f   observer = %s rad/s\n', ...
            S.mot(i).name, mat2str(g.Kx, 4), g.ki, g.beta_star, mat2str(g.p_obs.', 4));
end
end

function on_close(fig, src)
% Closing either window closes both and stops the timer
if isvalid(fig)
    S = guidata(fig);
    if isfield(S, 'tmr') && isvalid(S.tmr), stop(S.tmr); delete(S.tmr); end
    if isfield(S, 'fig2') && isvalid(S.fig2), delete(S.fig2); end
    delete(fig);
end
if isvalid(src), delete(src); end
end

%% ========================= computation ==================================
function p = poles_A(x)
% x = [wn, zeta, alpha] -> the three poles (column), as in design_A.m
wn = x(1);   zeta = min(x(2), 0.999);   alpha = x(3);
sigma = zeta*wn;   wd = wn*sqrt(1 - zeta^2);
p = [-sigma + 1i*wd;  -sigma - 1i*wd;  -alpha*sigma];
end

function E = eval_fast(m, Gp, S, g)
% Rational closed loop (Pade delay) with inputs [rN; rI; d] (same as explore_poles.m):
%   rN = reference through N (u = rN - Kx*xhat - ki*eI),  rI = reference into eI,  d = load
% One step simulation gives both beta:  r -> y = beta*k1*(rN -> y) + (rI -> y)
A = m.A;  B = m.B;  C = m.C;  P = Gp;  n = size(P.A, 1);  R = S.req.R;
Ac  = [A - B*g.Kx - g.L*C, -B*g.ki;  zeros(1,3)];      % observer + integrator
By  = [g.L; -1];   BrN = [B; 0];   BrI = [0; 0; 1];   Cc = [-g.Kx, -g.ki];
Acl = [P.A, P.B*Cc;  By*P.C, Ac];
Bcl = [P.B, zeros(n,1), P.B;  BrN, BrI, zeros(3,1)];
Ccl = [P.C, zeros(1,3);  zeros(1,n), Cc];
Dcl = [0 0 0;  1 0 0];
Y   = step(ss(Acl, Bcl, Ccl, Dcl), S.tf);              % Y(t, [y u], [rN rI d])
b   = [1, g.beta_star];                                % beta = 1, beta*
E.t = S.tf;
for k = 1:2
    E.y(:,k) = R*(b(k)*g.k1*Y(:,1,1) + Y(:,1,2));
    E.u(:,k) = R*(b(k)*g.k1*Y(:,2,1) + Y(:,2,2));
end
E.yd = S.req.d*Y(:,1,3);
Cy   = ss(Ac, By, Cc, 0);                              % controller y -> u
E.L  = -Cy*P;                                          % loop at the motor input
[gm, E.PM, ~, E.wc] = margin(E.L);   E.GM = 20*log10(gm);
E.noise = max(abs(squeeze(freqresp(Cy, S.wnn))))*S.req.q;
E.Ms = max(abs(1./(1 + squeeze(freqresp(E.L, S.w)))));  % peak of |S| (Pade delay)
E = step_metrics(E, S.req);
end

function E = eval_exact(m, S, g)
% Exact loop (true delay) with closed_loop, one call per beta
b = [1, g.beta_star];
for k = 1:2
    r = closed_loop(m, S.req, g, b(k));
    E.y(:,k) = r.y_r;   E.u(:,k) = r.u_r;
    if k == 2, rs = r; end
end
E.t = S.req.t;   E.yd = rs.y_d;   E.L = rs.L;
E.PM = rs.PM;   E.GM = rs.GM;   E.wc = rs.wc;   E.noise = rs.noise;   E.Ms = rs.Ms;
E = step_metrics(E, S.req);
end

function E = step_metrics(E, req)
% Overshoot, settling time and voltage peak of each step, IAE of the load step
for k = 1:2
    E.OS(k) = max(0, (max(E.y(:,k)) - req.R)/req.R*100);
    out = find(abs(E.y(:,k) - req.R) > 0.02*req.R, 1, 'last');
    E.ts(k) = 0;  if ~isempty(out), E.ts(k) = E.t(out); end
    E.up(k) = max(abs(E.u(:,k)));
end
E.IAE = trapz(E.t, abs(E.yd));
end

%% ========================= drawing ======================================
function update(fig, mode)
% Compute both motors (fast or exact) and update the plots and the STATS window
S = guidata(fig);   h = S.h;   st = S.s;   req = S.req;
p = poles_A(S.x);
for i = 1:2
    g{i} = pp_gains(S.mot(i), p);                                          %#ok<AGROW>
    if strcmp(mode, 'fast'), E{i} = eval_fast(S.mot(i), S.Gp{i}, S, g{i}); %#ok<AGROW>
    else,                    E{i} = eval_exact(S.mot(i), S, g{i});         %#ok<AGROW>
    end
end

% PLOTS window
set(h.poles, 'XData', real(p), 'YData', imag(p));
for i = 1:2
    for k = 1:2
        set(h.y(i,k), 'XData', E{i}.t, 'YData', E{i}.y(:,k));
        set(h.u(i,k), 'XData', E{i}.t, 'YData', E{i}.u(:,k));
    end
    set(h.d(i), 'XData', E{i}.t, 'YData', E{i}.yd);
    fr = squeeze(freqresp(E{i}.L, S.w));
    set(h.bm(i), 'XData', S.w, 'YData', 20*log10(abs(fr)));
    set(h.bp(i), 'XData', S.w, 'YData', unwrap(angle(fr))*180/pi);
end
h.ax.p.Title.String = sprintf(['\\omega_n = %.1f rad/s,  \\zeta = %.3f,  \\alpha = %.4f  ' ...
                               '(real pole %.2f rad/s)'], S.x(1), S.x(2), S.x(3), S.x(3)*S.x(2)*S.x(1));
h.ax.bm.Title.String = sprintf('PM %.1f / %.1f deg,  \\omega_c %.1f / %.1f rad/s  (3165 / 3166)', ...
                               E{1}.PM, E{2}.PM, E{1}.wc, E{2}.wc);
if strcmp(mode, 'fast'), h.mode.String = 'fast model (moving)'; else, h.mode.String = 'exact'; end

% STATS window: numbers of both motors
txt = {sprintf('poles: %s rad/s   observer: %s rad/s', ...
                 pole_str(p), mat2str(round(g{1}.p_obs.')))};
for i = 1:2
    txt{end+1} = sprintf(['motor %s: k1 = %.4f  k2 = %.5f  ki = %.4f   beta* = %.3f   ' ...
                            'GM = %.1f dB  chatter = %.3f V  IAE = %.3f'], ...
                           S.mot(i).name, g{i}.k1, g{i}.k2, g{i}.ki, g{i}.beta_star, ...
                           E{i}.GM, E{i}.noise, E{i}.IAE);                        %#ok<AGROW>
end
txt{end+1} = ['evaluation: ' h.mode.String];
st.info.String = txt;

% STATS window: requirement table, both motors (PID, beta*, beta = 1)
D = cell(numel(st.names), 8);
D(:,1) = st.names;   D(:,2) = st.lims;
for i = 1:2
    mk = @(k) struct('OS', E{i}.OS(k), 'ts', E{i}.ts(k), 'u_peak', E{i}.up(k), ...
                     'PM', E{i}.PM, 'noise', E{i}.noise, 'Ms', E{i}.Ms);
    [vs, xs] = pp_check(mk(2), p, req);                    % beta*
    [v1, x1] = pp_check(mk(1), p, req);                    % beta = 1
    c0 = 2 + 3*(i - 1);                                    % first column of this motor
    for j = 1:numel(st.names)
        D{j, c0+1} = cell_str(st.pid_x(j,i), st.pid_v(j,i));
        D{j, c0+2} = cell_str(xs(j), vs(j));
        D{j, c0+3} = cell_str(x1(j), v1(j));
    end
end
st.table.Data = D;
drawnow limitrate
end

function s = cell_str(x, v)
% value, with '*' when the requirement is not met (v > 0)
if v > 0, s = sprintf('%.2f *', x); else, s = sprintf('%.2f', x); end
end

function sync_controls(S)
% Put the current wn, zeta, alpha in the sliders and boxes
names = {'wn', 'zeta', 'alpha'};
fmt   = {'%.2f', '%.3f', '%.4f'};
for k = 1:3
    r = S.rng.(names{k});
    S.s.sl(k).Value  = min(max(S.x(k), r(1)), r(2));
    S.s.ed(k).String = sprintf(fmt{k}, S.x(k));
end
S.s.lock.Value = double(S.lock);
onoff = {'on', 'off'};
S.s.sl(3).Enable = onoff{1 + S.lock};       % alpha is computed when locked
S.s.ed(3).Enable = onoff{1 + S.lock};
end

function h = init_plots(fig, S)
% PLOTS window: pole map (left), step | voltage, load step, |L| | phase (right)
req = S.req;   t = req.t;   gry = [0.6 0.6 0.6];   nv = {'HandleVisibility', 'off'};
col = {[0 0.45 0.74], [0.85 0.33 0.10]};                % motor 3165, motor 3166
sty = {'-', '--'};                                       % beta = 1, beta*
tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
h.ax.p  = nexttile(tl, 1, [3 1]);
h.ax.y  = nexttile(tl, 2);    h.ax.u  = nexttile(tl, 3);
h.ax.d  = nexttile(tl, 5, [1 2]);
h.ax.bm = nexttile(tl, 8);    h.ax.bp = nexttile(tl, 9);

% Pole map (the observer poles are further left, see the STATS window)
ax = h.ax.p;  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on')
Lm = 1.15*max(req.p_max, S.rng.wn(2));   z = 0.78;   s = linspace(0, Lm, 2);
plot(ax, -z*s, sqrt(1 - z^2)*s, 'k:', -z*s, -sqrt(1 - z^2)*s, 'k:', nv{:});
xline(ax, -req.lam_min, 'b:', nv{:});
plot(ax, real(S.mot(1).p_pid), imag(S.mot(1).p_pid), 'x', 'Color', gry, 'MarkerSize', 11, 'LineWidth', 2);
plot(ax, [0, -1/S.mot(1).tau, -1/S.mot(2).tau], [0 0 0], 'g+', 'MarkerSize', 12, 'LineWidth', 2);
h.poles = plot(ax, nan, nan, 'o', 'MarkerSize', 11, 'LineWidth', 2, ...
               'MarkerFaceColor', [0.47 0.67 0.19], 'MarkerEdgeColor', 'k');
xlim(ax, [-Lm, 0.05*Lm]);   ylim(ax, [-0.6 0.6]*Lm);
xlabel(ax, 'Re [rad/s]');   ylabel(ax, 'Im [rad/s]');
legend(ax, {'PID poles', 'plant poles', 'controller poles'}, 'Location', 'southwest');
text(ax, 0.02, 0.02, 'dotted: \zeta = 0.78 (2% OS)   blue dotted: lam\_min', 'Units', 'normalized', ...
     'VerticalAlignment', 'bottom', 'Color', [0.4 0.4 0.4], 'FontSize', 8);
h.mode = text(ax, 0.98, 0.98, '', 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
              'VerticalAlignment', 'top', 'Color', [0.4 0.4 0.4]);

% Step and voltage: PID (grey), each motor with beta = 1 (solid) and beta* (dashed)
ax = h.ax.y;  hold(ax, 'on');  grid(ax, 'on')
for i = 1:2, plot(ax, t, S.pid{i}.y_r, 'Color', gry, nv{:}); end
for i = 1:2, for k = 1:2, h.y(i,k) = plot(ax, nan, nan, sty{k}, 'Color', col{i}, 'LineWidth', 1.3); end, end
yline(ax, req.R*[0.98 1.02], 'k:', nv{:});
legend(ax, [h.y(1,1), h.y(2,1), h.y(1,2)], {'3165', '3166', 'dashed: \beta*'}, 'Location', 'southeast');
title(ax, sprintf('%g deg step (grey: PID)', req.R));
xlabel(ax, 't [s]');  ylabel(ax, '\theta [deg]');  xlim(ax, [0 1]);  ylim(ax, [0 1.4]*req.R);

ax = h.ax.u;  hold(ax, 'on');  grid(ax, 'on')
for i = 1:2, plot(ax, t, S.pid{i}.u_r, 'Color', gry); end
for i = 1:2, for k = 1:2, h.u(i,k) = plot(ax, nan, nan, sty{k}, 'Color', col{i}, 'LineWidth', 1.3); end, end
yline(ax, [-1 1]*req.Vmax, 'r--');
title(ax, 'voltage on the step');  xlabel(ax, 't [s]');  ylabel(ax, 'u [V]');
xlim(ax, [0 0.5]);  ylim(ax, [-1.5 1.5]*req.Vmax);

% Load step (does not depend on beta)
ax = h.ax.d;  hold(ax, 'on');  grid(ax, 'on')
for i = 1:2, plot(ax, t, S.pid{i}.y_d, 'Color', gry); end
for i = 1:2, h.d(i) = plot(ax, nan, nan, 'Color', col{i}, 'LineWidth', 1.5); end
title(ax, sprintf('%.1f V load step (grey: PID)', req.d));
xlabel(ax, 't [s]');  ylabel(ax, '\theta [deg]');

% Bode of the loop at the motor input
ax = h.ax.bm;  set(ax, 'XScale', 'log');  hold(ax, 'on');  grid(ax, 'on')
for i = 1:2
    fr = squeeze(freqresp(S.pid{i}.L, S.w));
    plot(ax, S.w, 20*log10(abs(fr)), 'Color', gry);
    h.bm(i) = plot(ax, nan, nan, 'Color', col{i}, 'LineWidth', 1.3);
end
yline(ax, 0, 'k:');   xlabel(ax, '\omega [rad/s]');   ylabel(ax, '|L| [dB]');
ax = h.ax.bp;  set(ax, 'XScale', 'log');  hold(ax, 'on');  grid(ax, 'on')
for i = 1:2
    fr = squeeze(freqresp(S.pid{i}.L, S.w));
    plot(ax, S.w, unwrap(angle(fr))*180/pi, 'Color', gry);
    h.bp(i) = plot(ax, nan, nan, 'Color', col{i}, 'LineWidth', 1.3);
end
yline(ax, -180, 'k:');   xlabel(ax, '\omega [rad/s]');   ylabel(ax, '\angle L [deg]');
title(ax, 'grey: PID');
end

function st = init_stats(fig2, fig, S)
% STATS window: sliders, lock, buttons, numbers, requirement table of both motors
nrm = {'Units', 'normalized', 'BackgroundColor', 'w', 'FontSize', 10};
labels = {'wn [rad/s]', 'zeta [-]', 'alpha [-]'};
names  = {'wn', 'zeta', 'alpha'};
for k = 1:3
    y0 = 0.93 - 0.06*(k - 1);
    r  = S.rng.(names{k});
    uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.02 y0 0.13 0.04], ...
              'HorizontalAlignment', 'left', 'String', labels{k});
    st.sl(k) = uicontrol(fig2, 'Style', 'slider', 'Units', 'normalized', ...
              'Position', [0.16 y0 0.62 0.04], 'Min', r(1), 'Max', r(2), 'Value', r(1), ...
              'SliderStep', [0.005 0.05], 'Callback', @(~, ~) on_slide(fig, k, true));
    addlistener(st.sl(k), 'ContinuousValueChange', @(~, ~) on_slide(fig, k, false));
    st.ed(k) = uicontrol(fig2, 'Style', 'edit', 'Units', 'normalized', 'FontSize', 10, ...
              'Position', [0.80 y0 0.17 0.04], 'Callback', @(~, ~) on_edit(fig, k));
end
st.lock = uicontrol(fig2, 'Style', 'checkbox', nrm{:}, 'Position', [0.02 0.74 0.55 0.04], ...
          'String', sprintf('lock alpha: real pole = %g rad/s  (alpha = %g/(zeta*wn))', S.lam, S.lam), ...
          'Callback', @(~, ~) on_lock(fig));
uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.58 0.735 0.12 0.04], ...
          'HorizontalAlignment', 'right', 'String', 'Simulink beta:');
st.beta = uicontrol(fig2, 'Style', 'popupmenu', 'Units', 'normalized', 'FontSize', 10, ...
          'Position', [0.71 0.74 0.26 0.04], 'String', {'beta = 1 (paths)', 'beta* (steps)'});
uicontrol(fig2, 'Style', 'pushbutton', 'Units', 'normalized', 'FontSize', 10, ...
          'Position', [0.02 0.68 0.46 0.05], 'String', 'Write to Simulink (both motors)', ...
          'Callback', @(~, ~) on_write(fig));
uicontrol(fig2, 'Style', 'pushbutton', 'Units', 'normalized', 'FontSize', 10, ...
          'Position', [0.51 0.68 0.46 0.05], 'String', 'Print poles and gains', ...
          'Callback', @(~, ~) on_print(fig));
st.info = uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.02 0.50 0.96 0.17], ...
          'HorizontalAlignment', 'left', 'FontName', 'Monospaced', 'FontSize', 8);

% Requirement rows of pp_check and the PID values of both motors (fixed)
for i = 1:2
    [vp, xp, lim, nm] = pp_check(S.pid{i}, S.mot(i).p_pid, S.req);
    st.pid_x(:,i) = xp;   st.pid_v(:,i) = vp;
end
st.names = nm;
mn = [false false false true false true false];            % phase margin and slowest pole: minimum
st.lims = cell(numel(nm), 1);
for j = 1:numel(nm)
    if mn(j), st.lims{j} = sprintf('>= %g', lim(j)); else, st.lims{j} = sprintf('<= %g', lim(j)); end
end
st.table = uitable(fig2, 'Units', 'normalized', 'Position', [0.02 0.02 0.96 0.46], ...
          'ColumnName', {'requirement', 'limit', '3165 PID', '3165 beta*', '3165 beta=1', ...
                         '3166 PID', '3166 beta*', '3166 beta=1'}, 'RowName', [], ...
          'ColumnWidth', {125, 55, 62, 70, 75, 62, 70, 75}, 'FontSize', 9, ...
          'Data', cell(numel(nm), 8));
end

function s = pole_str(p)
% e.g. '-45.0 +/- 21.8j,  -3.0'
p = p(:);   used = false(size(p));   parts = {};
for i = 1:numel(p)
    if used(i), continue; end
    if abs(imag(p(i))) > 1e-9
        parts{end+1} = sprintf('%.1f +/- %.1fj', real(p(i)), abs(imag(p(i)))); %#ok<AGROW>
        used(abs(p - conj(p(i))) < 1e-9) = true;
    else
        parts{end+1} = sprintf('%.2f', real(p(i)));                           %#ok<AGROW>
    end
    used(i) = true;
end
s = strjoin(parts, ',  ');
end
