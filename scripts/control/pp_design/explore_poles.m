%% EXPLORE_POLES - move the closed-loop poles by hand and watch the loop change
% Two windows:
%   PLOTS : pole map (drag the poles), 10 deg step, voltage, load step, Bode of the loop
%   STATS : type the poles by hand, poles/gains, requirement table (PID, beta* and beta = 1;
%           the chatter row is left out, see st.rows in init_stats)
% Mouse : drag a pole (left button). A complex pole drags its conjugate too.
% Keys  : (in the PLOTS window)  'c' -> pair + 1 real pole   'r' -> 3 real poles
%                                'p' -> PID poles            'g' -> print poles and gains
% By hand: in STATS write e.g.  -31.2+11.1j, -7   or   -9.7, -21.1, -35.9   and press Enter
%          (the conjugate of a complex pole is added automatically).
% Step plot: beta = 1 (no weight, the choice for trajectories) and beta* (zero on the
%            slowest real pole, the choice for steps).
% Smooth dragging: while dragging a FAST model is used (Pade delay, one simulation for
% all beta, timer refresh, frames dropped if busy); on release everything is EXACT.
% Moving the poles (STATS prints wn, zeta, alpha of the pair):
%   further LEFT  (larger wn)    -> faster and stiffer joint, more voltage and encoder noise
%   closer to the REAL AXIS (zeta -> 1) -> less oscillation/overshoot, slightly slower
%   real pole further left (larger alpha) -> faster integral action: friction and load
%                                errors removed sooner, but more overshoot with beta = 1
clear; clc; close all
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));
motor   = 2;               % 1 = motor 3165, 2 = motor 3166
fps     = 20;              % max plot refreshes per second while dragging

%% Model, PID reference and settings
[mot, req] = pp_setup();
S.m       = mot(motor);
S.req     = req;
S.pid     = closed_loop(S.m, req, S.m.Cpid);      % exact, once
S.p_pid   = S.m.p_pid;
S.p       = S.p_pid;           % start from the PID poles
S.drag    = 0;                 % index of the pole being dragged (0 = none)
S.dirty   = false;             % true = poles moved, plots to refresh

% Fast model used while dragging: delay replaced by a Pade approximation
Gd = S.m.G;   Gd.InputDelay = req.Td;
S.Gp = ss(pade(Gd, 2));
S.tf = linspace(0, req.t(end), 750)';           % coarser time grid
S.w  = logspace(0, log10(pi/req.Ts), 200);      % Bode frequencies
S.wn = logspace(2, log10(pi/req.Ts), 50);       % chatter frequencies

%% Windows and all graphic objects (created once, then only updated)
scr  = get(0, 'ScreenSize');
fig  = figure('Name', sprintf('PLOTS - motor %s - drag the poles', S.m.name), 'Units', 'normalized', ...
              'Position', [0.01 0.05 0.68 0.87], 'Color', 'w');
fig2 = figure('Name', 'STATS - poles and requirements', 'Units', 'pixels', ...
              'Position', [scr(3) - 500, 80, 480, 570], 'Color', 'w', 'MenuBar', 'none', 'ToolBar', 'none');
S.h = init_plots(fig, S);
S.s = init_stats(fig2, fig, S);
set(fig, 'WindowButtonDownFcn', @on_down, 'WindowButtonUpFcn', @on_up, ...
         'WindowButtonMotionFcn', @on_move, 'KeyPressFcn', @on_key, ...
         'CloseRequestFcn', @(src, ~) on_close(fig, src));
set(fig2, 'CloseRequestFcn', @(src, ~) on_close(fig, src));
S.fig2 = fig2;
S.tmr  = timer('ExecutionMode', 'fixedSpacing', 'Period', round(1000/fps)/1000, ...
               'BusyMode', 'drop', 'TimerFcn', @(~, ~) on_tick(fig));
guidata(fig, S);
draw_poles(S);
update(fig, 'exact');
start(S.tmr);
figure(fig);

%% ========================= callbacks ====================================
function on_down(fig, ~)
% Pick the pole closest to the mouse (within 6% of the axis size)
S  = guidata(fig);
cp = S.h.ax.p.CurrentPoint(1, 1:2);
sc = [diff(S.h.ax.p.XLim), diff(S.h.ax.p.YLim)];
d  = hypot((real(S.p) - cp(1))/sc(1), (abs(imag(S.p)) - abs(cp(2)))/sc(2));
[dmin, i] = min(d);
if dmin < 0.06, S.drag = i;  guidata(fig, S);  end
end

function on_move(fig, ~)
% Cheap: move the pole, redraw only the markers, ask the timer for a refresh
S = guidata(fig);
if S.drag == 0, return; end
cp = S.h.ax.p.CurrentPoint(1, 1:2);
re = min(cp(1), -0.5);                       % keep the pole stable
i  = S.drag;
if abs(imag(S.p(i))) > 0                     % complex: move the pair together
    j  = find(abs(S.p - conj(S.p(i))) < 1e-9 & (1:3)' ~= i, 1);
    im = max(abs(cp(2)), 0.5);
    S.p(i) = re + 1j*im*sign(imag(S.p(i)));
    S.p(j) = conj(S.p(i));
else                                         % real: stay on the real axis
    S.p(i) = re;
end
S.dirty = true;
guidata(fig, S);
draw_poles(S);
end

function on_tick(fig)
% Timer: refresh the plots with the fast model if the poles moved
if ~isvalid(fig) || strcmp(fig.BeingDeleted, 'on'), return; end
S = guidata(fig);
if ~S.dirty, return; end
S.dirty = false;   guidata(fig, S);
update(fig, 'fast');
end

function on_up(fig, ~)
S = guidata(fig);
if S.drag == 0, return; end
S.drag = 0;   S.dirty = false;   guidata(fig, S);
update(fig, 'exact');
print_poles(S);
end

function on_key(fig, evt)
S = guidata(fig);   p = S.p;   isr = abs(imag(p)) < 1e-9;
switch evt.Character
    case 'c'                                 % 3 real -> pair (from the 2 fastest) + real
        if all(isr)
            p  = sort(real(p));              % most negative first
            re = mean(p(1:2));   im = max(abs(diff(p(1:2)))/2, 0.3*abs(re));
            S.p = [re + 1j*im; re - 1j*im; p(3)];
        end
    case 'r'                                 % pair -> 2 real poles around its real part
        if ~all(isr)
            pc  = p(~isr);   re = real(pc(1));
            S.p = sort([0.8*re; 1.25*re; real(p(isr))]);
        end
    case 'p'
        S.p = S.p_pid;
    case 'g'
        print_poles(S);   return
    otherwise
        return
end
set_poles(fig, S.p);
end

function on_typed(fig)
% Poles written by hand in the STATS window
S = guidata(fig);
[p, msg] = parse_poles(S.s.edit.String);
if isempty(p)
    set(S.s.status, 'String', msg, 'ForegroundColor', [0.8 0 0]);   return
end
set(S.s.status, 'String', 'poles applied', 'ForegroundColor', [0 0.5 0]);
set_poles(fig, p);
end

function set_poles(fig, p)
% New pole set (keys or typed): redraw, exact update, print
S = guidata(fig);   S.p = p(:);   S.dirty = false;   guidata(fig, S);
draw_poles(S);   update(fig, 'exact');   print_poles(S);
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
function E = eval_fast(S, g)
% Rational closed loop (Pade delay) with inputs [rN; rI; d]:
%   rN = reference through N (u = rN - Kx*xhat - ki*eI),  rI = reference into eI,  d = load
% One step simulation gives both beta:  r -> y = beta*k1*(rN -> y) + (rI -> y)
A = S.m.A;  B = S.m.B;  C = S.m.C;  P = S.Gp;  n = size(P.A, 1);  R = S.req.R;
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
E.noise = max(abs(squeeze(freqresp(Cy, S.wn))))*S.req.q;
E.Ms = max(abs(1./(1 + squeeze(freqresp(E.L, S.w)))));  % peak of |S| (Pade delay)
E = step_metrics(E, S.req);
end

function E = eval_exact(S, g)
% Exact loop (true delay) with closed_loop, one call per beta
b = [1, g.beta_star];
for k = 1:2
    r = closed_loop(S.m, S.req, g, b(k));
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

function [p, msg] = parse_poles(s)
% Text -> 3 poles. Accepts '-31.2+11.1j, -7'  '-9.7 -21.1 -35.9'  '[-20 -20 -20]'.
% A complex pole without its conjugate gets it added.
p = [];   msg = '';
s = regexprep(s, '(\d)\s*([+-])\s*(\d[\d.]*[ij])', '$1$2$3');    % '-31 + 11j' -> '-31+11j'
v = str2num(['[' s ']']);                %#ok<ST2NM>  (your own input, evaluated as numbers)
if isempty(v) || ~isnumeric(v), msg = 'could not read the numbers';  return; end
v = v(:);
for k = 1:numel(v)                       % add missing conjugates
    if abs(imag(v(k))) > 1e-9 && ~any(abs(v - conj(v(k))) < 1e-9)
        v(end+1) = conj(v(k));           %#ok<AGROW>
    end
end
if numel(v) ~= 3
    msg = sprintf('need 3 poles, got %d (a complex pole counts twice)', numel(v));  return
end
if any(real(v) >= 0), msg = 'all poles need a negative real part';  return; end
p = v;
end

%% ========================= drawing ======================================
function update(fig, mode)
% Compute (fast or exact) and update the existing graphic objects of both windows
S = guidata(fig);   h = S.h;   st = S.s;   req = S.req;
g = pp_gains(S.m, S.p);
if strcmp(mode, 'fast'), E = eval_fast(S, g); else, E = eval_exact(S, g); end
bs = g.beta_star;

% PLOTS window
for k = 1:2
    set(h.y(k), 'XData', E.t, 'YData', E.y(:,k));
    set(h.u(k), 'XData', E.t, 'YData', E.u(:,k));
end
set(h.d, 'XData', E.t, 'YData', E.yd);
h.ax.y.Title.String = sprintf('step %g deg: OS %.1f%% (\\beta=1), %.1f%% (\\beta*)', ...
                              req.R, E.OS(1), E.OS(2));
h.leg_y.String{3} = sprintf('\\beta* = %.2f', bs);
h.ax.d.Title.String = sprintf('%.1f V load step: IAE %.3f (PID %.3f)', req.d, E.IAE, S.pid.IAE);
fr = squeeze(freqresp(E.L, S.w));
set(h.bm, 'XData', S.w, 'YData', 20*log10(abs(fr)));
set(h.bp, 'XData', S.w, 'YData', unwrap(angle(fr))*180/pi);
if isfinite(E.wc), h.wc1.Value = E.wc;   h.wc2.Value = E.wc;  end   % no crossover -> keep
h.ax.bm.Title.String = sprintf('loop: \\omega_c %.1f rad/s, PM %.1f deg, GM %.1f dB', E.wc, E.PM, E.GM);
if strcmp(mode, 'fast'), h.mode.String = 'fast model (dragging)'; else, h.mode.String = 'exact'; end

% STATS window: numbers
st.info.String = { ...
    ['poles    : ' pole_str(S.p) '  rad/s'], ...
    ['           ' pair_str(S.p)], ...
    sprintf('gains    : k1 = %.4f   k2 = %.5f   ki = %.4f', g.k1, g.k2, g.ki), ...
    sprintf('beta*    : %.3f   ->  N = beta*k1 = %.4f', bs, bs*g.k1), ...
    sprintf('observer : %s rad/s', mat2str(round(g.p_obs.'))), '', ...
    '                 beta=1   beta*', ...
    sprintf('overshoot  [%%] %7.1f %7.1f', E.OS), ...
    sprintf('settling   [s] %7.2f %7.2f', E.ts), ...
    sprintf('u peak     [V] %7.1f %7.1f', E.up), '', ...
    sprintf('loop: wc = %.1f rad/s   PM = %.1f deg   GM = %.1f dB', E.wc, E.PM, E.GM), ...
    sprintf('load step IAE = %.3f (PID %.3f)  chatter = %.3f V', E.IAE, S.pid.IAE, E.noise), ...
    ['evaluation: ' h.mode.String]};

% STATS window: requirement table, beta* and beta = 1
mk = @(k) struct('OS', E.OS(k), 'ts', E.ts(k), 'u_peak', E.up(k), 'PM', E.PM, 'noise', E.noise, 'Ms', E.Ms);
[vs, xs] = pp_check(mk(2), S.p, req);
[v1, x1] = pp_check(mk(1), S.p, req);
vs = vs(st.rows);   xs = xs(st.rows);                      % keep only the rows shown
v1 = v1(st.rows);   x1 = x1(st.rows);

ok = {'NO', 'ok'};
D  = cell(numel(st.rows), 8);
for k = 1:numel(st.rows)
    D(k,:) = {st.names{k}, st.lims{k}, ...
              sprintf('%.2f', st.pid_x(k)), ok{1 + (st.pid_v(k) <= 0)}, ...   % PID
              sprintf('%.2f', xs(k)),       ok{1 + (vs(k) <= 0)}, ...         % beta*
              sprintf('%.2f', x1(k)),       ok{1 + (v1(k) <= 0)}};            % beta = 1
end
st.table.Data = D;
st.sumA.String = sprintf('beta*: %d/%d met', sum(vs <= 0), numel(vs));
st.sumB.String = sprintf('beta = 1: %d/%d met', sum(v1 <= 0), numel(v1));
if strcmp(mode, 'exact'), st.edit.String = edit_str(S.p); end
drawnow limitrate
end

function draw_poles(S)
% Move the pole markers and their labels (cheap, called on every mouse move)
h = S.h;
set(h.poles, 'XData', real(S.p), 'YData', imag(S.p));
dx = 0.02*diff(h.ax.p.XLim);
for k = 1:3
    if abs(imag(S.p(k))) < 1e-9, s = sprintf('%.1f', real(S.p(k)));
    else, s = sprintf('%.1f%+.1fj', real(S.p(k)), imag(S.p(k))); end
    set(h.plab(k), 'Position', [real(S.p(k)) + dx, imag(S.p(k)) + dx, 0], 'String', s);
end
drawnow limitrate
end

function h = init_plots(fig, S)
% PLOTS window: pole map (left), step | voltage, load step, |L| | phase (right)
req = S.req;   t = req.t;   gry = [0.6 0.6 0.6];   nv = {'HandleVisibility', 'off'};
tl = tiledlayout(fig, 3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
h.ax.p  = nexttile(tl, 1, [3 2]);
h.ax.y  = nexttile(tl, 3);    h.ax.u  = nexttile(tl, 4);
h.ax.d  = nexttile(tl, 7, [1 2]);
h.ax.bm = nexttile(tl, 11);   h.ax.bp = nexttile(tl, 12);

% Pole map
ax = h.ax.p;  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on')
Lm = 1.15*req.p_max;   z = 0.78;   s = linspace(0, Lm, 2);       % zeta = 0.78 -> 2% OS
plot(ax, -z*s, sqrt(1 - z^2)*s, 'k:', -z*s, -sqrt(1 - z^2)*s, 'k:', nv{:});
th = linspace(pi/2, 3*pi/2, 100);
plot(ax, req.wn_max*cos(th), req.wn_max*sin(th), 'k-.', nv{:});
xline(ax, -req.p_max, 'k--', nv{:});
xline(ax, -req.lam_min, 'b:', nv{:});
plot(ax, real(S.p_pid), imag(S.p_pid), 'x', 'Color', [0.5 0.5 0.5], 'MarkerSize', 11, 'LineWidth', 2);
plot(ax, [0 -1/S.m.tau], [0 0], 'g+', 'MarkerSize', 12, 'LineWidth', 2);
h.poles = plot(ax, nan, nan, 'o', 'MarkerSize', 11, 'LineWidth', 2, ...
               'MarkerFaceColor', [0.85 0.33 0.10], 'MarkerEdgeColor', 'k');
for k = 1:3, h.plab(k) = text(ax, 0, 0, '', 'FontSize', 9, 'FontWeight', 'bold'); end
xlim(ax, [-Lm, 0.05*Lm]);   ylim(ax, [-0.6 0.6]*Lm);
xlabel(ax, 'Re [rad/s]');   ylabel(ax, 'Im [rad/s]');
title(ax, {'drag the poles (keys: c = pair, r = real, p = PID, g = print)', ...
           'dotted: \zeta = 0.78 (2% OS)   blue dotted: slowest pole allowed   dash-dot: 1.5/\tau   dashed: 3/\tau'});
legend(ax, {'PID poles', 'plant poles', 'controller poles'}, 'Location', 'southwest');
h.mode = text(ax, 0.98, 0.98, '', 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
              'VerticalAlignment', 'top', 'Color', [0.4 0.4 0.4]);

% Reference step and voltage: PID (static) + beta = 1, beta*
sty = {{'--'}, {'-', 'LineWidth', 1.5}};
ax = h.ax.y;  hold(ax, 'on');  grid(ax, 'on')
hp = plot(ax, t, S.pid.y_r, 'Color', gry);
for k = 1:2, h.y(k) = plot(ax, nan, nan, sty{k}{:}); end
yline(ax, req.R*[0.98 1.02], 'k:', nv{:});
h.leg_y = legend(ax, [hp, h.y], {'PID', '\beta = 1', '\beta*'}, 'Location', 'southeast');
xlabel(ax, 't [s]');  ylabel(ax, '\theta [deg]');  xlim(ax, [0 1]);  ylim(ax, [0 1.4]*req.R);

ax = h.ax.u;  hold(ax, 'on');  grid(ax, 'on')
plot(ax, t, S.pid.u_r, 'Color', gry);
for k = 1:2, h.u(k) = plot(ax, nan, nan, sty{k}{:}); end
yline(ax, [-1 1]*req.Vmax, 'r--');
title(ax, 'voltage on the step');  xlabel(ax, 't [s]');  ylabel(ax, 'u [V]');
xlim(ax, [0 0.5]);  ylim(ax, [-1.5 1.5]*req.Vmax);

% Load step (does not depend on beta)
ax = h.ax.d;  hold(ax, 'on');  grid(ax, 'on')
plot(ax, t, S.pid.y_d, 'Color', gry);
h.d = plot(ax, nan, nan, 'LineWidth', 1.5);
xlabel(ax, 't [s]');  ylabel(ax, '\theta [deg]');  legend(ax, 'PID', 'pole placement');

% Bode of the loop at the motor input
fr = squeeze(freqresp(S.pid.L, S.w));
ax = h.ax.bm;  set(ax, 'XScale', 'log');  hold(ax, 'on');  grid(ax, 'on')
plot(ax, S.w, 20*log10(abs(fr)), 'Color', gry);
h.bm = plot(ax, nan, nan, 'LineWidth', 1.5);
yline(ax, 0, 'k:');   h.wc1 = xline(ax, 1, ':');
xlabel(ax, '\omega [rad/s]');   ylabel(ax, '|L| [dB]');
ax = h.ax.bp;  set(ax, 'XScale', 'log');  hold(ax, 'on');  grid(ax, 'on')
plot(ax, S.w, unwrap(angle(fr))*180/pi, 'Color', gry);
h.bp = plot(ax, nan, nan, 'LineWidth', 1.5);
yline(ax, -180, 'k:');   h.wc2 = xline(ax, 1, ':');
xlabel(ax, '\omega [rad/s]');   ylabel(ax, '\angle L [deg]');   title(ax, 'grey: PID');
end

function st = init_stats(fig2, fig, S)
% STATS window: pole entry, numbers, requirement table
nrm = {'Units', 'normalized', 'BackgroundColor', 'w', 'FontSize', 10};
uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.03 0.94 0.94 0.045], ...
          'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
          'String', 'Poles by hand (Enter or Apply):');
st.edit = uicontrol(fig2, 'Style', 'edit', 'Units', 'normalized', 'FontSize', 11, ...
          'Position', [0.03 0.89 0.70 0.05], 'HorizontalAlignment', 'left', ...
          'Callback', @(~, ~) on_typed(fig));
uicontrol(fig2, 'Style', 'pushbutton', 'Units', 'normalized', 'FontSize', 10, ...
          'Position', [0.75 0.89 0.22 0.05], 'String', 'Apply', 'Callback', @(~, ~) on_typed(fig));
st.status = uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.03 0.845 0.94 0.04], ...
          'HorizontalAlignment', 'left', 'String', 'e.g.  -31.2+11.1j, -7    or    -9.7, -21.1, -35.9');
st.info = uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.03 0.42 0.94 0.42], ...
          'HorizontalAlignment', 'left', 'FontName', 'Monospaced', 'FontSize', 8);
st.sumA = uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.03 0.375 0.46 0.04], ...
          'HorizontalAlignment', 'left', 'FontWeight', 'bold');
st.sumB = uicontrol(fig2, 'Style', 'text', nrm{:}, 'Position', [0.51 0.375 0.46 0.04], ...
          'HorizontalAlignment', 'left', 'FontWeight', 'bold');
% Requirement rows of pp_check, and the PID values (computed once, the PID does not move)
[vp, xp, lim, names] = pp_check(S.pid, S.p_pid, S.req);
% rows shown: 1 overshoot, 2 settling, 3 peak voltage, 4 phase margin, 6 slowest pole, 7 Ms.
% Row 5 (chatter) is left out of this table: write [1 2 3 4 5 6 7] to show it again.
st.rows  = [1 2 3 4 6 7];
st.names = names(st.rows);
st.pid_x = xp(st.rows);                                    % PID values
st.pid_v = vp(st.rows);                                    % PID violations (<= 0: met)
mn = [false false false true false true false];            % phase margin and slowest pole: minimum
st.lims = cell(1, numel(st.rows));
for k = 1:numel(st.rows)
    j = st.rows(k);
    if mn(j), st.lims{k} = sprintf('>= %g', lim(j)); else, st.lims{k} = sprintf('<= %g', lim(j)); end
end
st.table = uitable(fig2, 'Units', 'normalized', 'Position', [0.03 0.02 0.94 0.35], ...
          'ColumnName', {'requirement', 'limit', 'PID', 'ok', 'beta*', 'ok', 'beta=1', 'ok'}, 'RowName', [], ...
          'ColumnWidth', {125, 52, 45, 28, 45, 28, 45, 28}, 'FontSize', 9, ...
          'Data', cell(numel(st.rows), 8));
end

%% ========================= printing =====================================
function print_poles(S)
% Current poles, pattern, gains and beta* in the command window
g = pp_gains(S.m, S.p);
fprintf('\npoles    : %s rad/s\n', pole_str(S.p));
fprintf('           %s\n', pair_str(S.p));
fprintf('gains    : k1 = %.4f  k2 = %.5f  ki = %.4f   beta* = %.3f\n', g.k1, g.k2, g.ki, g.beta_star);
fprintf('observer : %s rad/s   L = %s\n', mat2str(g.p_obs.', 4), mat2str(g.L.', 5));
end

function s = pole_str(p)
% e.g. '-31.2 +/- 11.1j,  -7.0'
p = p(:);   used = false(size(p));   parts = {};
for i = 1:numel(p)
    if used(i), continue; end
    if abs(imag(p(i))) > 1e-9
        parts{end+1} = sprintf('%.1f +/- %.1fj', real(p(i)), abs(imag(p(i)))); %#ok<AGROW>
        used(abs(p - conj(p(i))) < 1e-9) = true;
    else
        parts{end+1} = sprintf('%.1f', real(p(i)));                           %#ok<AGROW>
    end
    used(i) = true;
end
s = strjoin(parts, ',  ');
end

function s = edit_str(p)
% Poles as text for the edit box (upper complex pole only: its conjugate is implied)
p = p(:);   parts = {};
for i = 1:numel(p)
    if abs(imag(p(i))) < 1e-9, parts{end+1} = sprintf('%.2f', real(p(i)));        %#ok<AGROW>
    elseif imag(p(i)) > 0,     parts{end+1} = sprintf('%.2f%+.2fj', real(p(i)), imag(p(i))); %#ok<AGROW>
    end
end
s = strjoin(parts, ', ');
end

function s = pair_str(p)
% wn, zeta and alpha of the complex pair, if there is one
pc = p(abs(imag(p)) > 1e-9);
if isempty(pc), s = 'all poles real'; return; end
wn = abs(pc(1));   zeta = -real(pc(1))/wn;
pr = real(p(abs(imag(p)) < 1e-9));
s  = sprintf('pair: wn = %.1f rad/s, zeta = %.3f, alpha = %.3f', wn, zeta, abs(pr(1))/(zeta*wn));
end
