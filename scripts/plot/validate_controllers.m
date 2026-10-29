%% VALIDATE CONTROLLERS - compare saved Simulink / hardware runs
% Workflow:
%   1) run the design script (pid.m or pole_placement.m)
%   2) run the Simulink model (simulation or CNC): the StopFcn callback calls
%      run_and_save, which saves <model>_<yyyymmdd>_<HHMMSS>.mat in
%      data/sim (simulation) or data/lab (bench run)
%   3) repeat for the other controllers, list the files below and run this script
%
% Each file contains (written by run_and_save):
%   - the To Workspace signals, with or without the _<tag> suffix
%       r_3165 y_3165 u_3165 usat_3165 d_3165   (same for 3166)   [deg, V]
%       xc_sim (or xc_real) yc_sim (or yc_real) xc_des yc_des     [m]
%   - the gains of the controller used in that run
%       PID: kp_xxxx ki_xxxx kd_xxxx        PP: Kx_xxxx Ki_xxxx A_obs_xxxx B_obs_xxxx
%   - the plant parameters k_xxxx_load, tau_xxxx_load (from parameters.m)
%
% From the MODEL (gains + k, tau): Ms, margins, Nyquist, Bode, |S|, |T|, root locus.
% From the DATA: joint/tip errors, command and saturation, disturbance and step KPIs.
% The feedforward is outside the loop: it does not change any robustness result.
clear; clc; close all

%% ===== Runs to compare (edit here) =====
runs = {
%  file: name only (searched in data/sim and data/lab), 'sim/...', 'lab/...' or full path
%  file                                       type    label
  'Model_pid_ff_20261029_120653.mat',         'PID',   '20wc'
  'Model_pid_ff_20261029_120803.mat',           'PID',   '30wc'
};

%% ===== Options =====
opt.N_pid     = 100;            % derivative filter of the Simulink PID block (default)
opt.enc_res   = 360/4096;       % [deg] one encoder count
opt.k_var     = 0.2;            % robustness check: k*(1 +- k_var)
opt.tau_var   = [0.021 0.045];  % robustness check: tau range [s]
opt.Vmax      = 6;              % [V] used if the file has no Vmax
opt.t_skip    = 1;              % [s] initial transient (home -> start of the path)
                                %     excluded from the tracking/contour errors

%% ===== Load and analyze =====
data_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'data');
motors   = {'3165', '3166'};
names    = runs(:,3)';
nR       = size(runs, 1);
% identical labels are allowed: they get a number, e.g. 'PP (2)'
for j = 1:nR
    same = find(strcmp(runs(:,3)', runs{j,3}));
    if numel(same) > 1
        names{j} = sprintf('%s (%d)', runs{j,3}, find(same == j));
    end
end

for j = 1:nR
    f = find_run(runs{j,1}, data_dir);
    fprintf('Run %d: %s\n', j, f);
    S = load(f);
    if isfield(S, 'Vmax'), opt.Vmax = S.Vmax; end
    for i = 1:2
        mdl(i,j) = model_kpi(S, runs{j,2}, motors{i}, opt);
        dat(i,j) = data_kpi(S, motors{i}, opt);
    end
    tip(j) = tip_kpi(S, min([dat(:,j).t_dist]), opt.t_skip);
end

%% ===== Tables =====
rows_model = {
  'Ms',              'Ms [-]'
  'Ms_worst',        'Ms worst (k,tau varied) [-]'
  'GM_from_Ms',      'GM guaranteed by Ms [dB]'
  'PM_from_Ms',      'PM guaranteed by Ms [deg]'
  'GM',              'GM, gain increase [dB] (Inf = none)'
  'PM',              'PM [deg]'
  'wc',              'Crossover wc [rad/s]'
  'delay_margin_ms', 'Delay margin [ms]'
  'noise',           'Encoder noise on u [V/count]'
  'slow_sigma',      'Slowest CL pole: decay [1/s]'
  'slow_zeta',       'Slowest CL pole: damping [-]'
};
rows_data = {
  'e_max',      'Joint error max [deg]'
  'e_rms',      'Joint error RMS [deg]'
  'u_peak',     'Peak |u| before saturation [V]'
  'sat',        'Time in saturation [%]'
  'dist_peak',  'Disturbance: peak error [deg]'
  'dist_rec',   'Disturbance: recovery [s]'
  'dist_IAE',   'Disturbance: IAE [deg s]'
  'rise',       'Step: rise time 10-90 [s]'
  'settling',   'Step: settling 2% [s]'
  'overshoot',  'Step: overshoot [%]'
};
rows_tip = {
  'contour_max', 'Contour error max [mm]'
  'contour_rms', 'Contour error RMS [mm]'
  'track_max',   'Tracking error max [mm]'
};
for i = 1:2
    print_table(['MODEL - motor ' motors{i}], rows_model, [mdl(i,:).kpi], names);
    print_table(['DATA - motor ' motors{i}],  rows_data,  [dat(i,:).kpi], names);
end
print_table('TIP (x, y)', rows_tip, [tip.kpi], names);
fprintf(['NaN = not available in that run (no disturbance / no step / not recovered).\n' ...
         'Tracking errors: from t_skip = %g s to the start of the disturbance.\n'], opt.t_skip);

%% ===== Plots =====
col = lines(nR);
for i = 1:2, plot_robustness(mdl(i,:), names, col, ['motor ' motors{i}]); end
plot_joints(dat, names, col, motors, opt);
plot_tests(dat, names, col, motors);
plot_tip(tip, names, col);


%% =====================================================================
%% LOCAL FUNCTIONS
%% =====================================================================

function m = model_kpi(S, type, mot, opt)
% Loop functions from the model: plant G (from k, tau) and feedback part of
% the controller Cfb (u = -Cfb*y, reference and feedforward set to zero).
G   = plant_ss(S.(['k_' mot '_load']), S.(['tau_' mot '_load']));
Cfb = feedback_controller(S, type, mot, opt);

L  = Cfb*G;
Sf = feedback(1, L);                 % sensitivity S
Tf = feedback(L, 1);                 % complementary sensitivity T
m.L = L;
m.w  = logspace(-2, 4, 3000);
m.Lw = squeeze(freqresp(L,  m.w));
m.Sw = squeeze(freqresp(Sf, m.w));
m.Tw = squeeze(freqresp(Tf, m.w));

k.Ms = getPeakGain(Sf);
k.GM_from_Ms = 20*log10(k.Ms/(k.Ms - 1));
k.PM_from_Ms = 2*asind(1/(2*k.Ms));
[~, Pm, ~, Wcp] = margin(L);
% gain margin = how much the gain can be INCREASED. With two integrators in
% the loop (PID + motor) the phase starts at -180 deg: margin() then reports
% a meaningless crossing at w -> 0 (GM = -Inf). Ignore crossings below wc/1000.
am = allmargin(L);
up = am.GainMargin > 1 & am.GMFrequency > 1e-3*Wcp;
if any(up), k.GM = 20*log10(min(am.GainMargin(up))); else, k.GM = Inf; end
k.PM = Pm;
k.wc = Wcp;
k.delay_margin_ms = 1e3*deg2rad(Pm)/Wcp;
k.noise = getPeakGain(feedback(Cfb, G))*opt.enc_res;     % |C S| max x one count

CL = feedback(G, Cfb);               % all closed-loop states (plant + controller)
m.cl_poles = pole(CL);
if ~isstable(CL), warning('Run with %s on motor %s: model loop UNSTABLE', type, mot); end
[wn, zeta] = damp(CL);
[k.slow_sigma, idx] = min(wn.*zeta);
k.slow_zeta = zeta(idx);

% same controller on a plant with k and tau varied
m.bundle = {};
Ms_all = k.Ms;
for a = [1 - opt.k_var, 1 + opt.k_var]
    for tau = opt.tau_var
        Gb = plant_ss(a*S.(['k_' mot '_load']), tau);
        Lb = Cfb*Gb;
        m.bundle{end+1} = squeeze(freqresp(Lb, m.w));
        if isstable(feedback(Gb, Cfb)), Ms_all(end+1) = getPeakGain(feedback(1, Lb));
        else,                           Ms_all(end+1) = Inf;
        end
    end
end
k.Ms_worst = max(Ms_all);
m.kpi = k;
end

function G = plant_ss(k, tau)
% V -> measured angle [deg, encoder sign]: G = k*(-180/pi) / (s (tau s + 1))
G = ss(tf(k*(-180/pi), [tau 1 0]));
end

function Cfb = feedback_controller(S, type, mot, opt)
% Feedback part of the controller, convention u = -Cfb*y
s = tf('s');
switch upper(type)
    case 'PID'      % Simulink PID block, parallel form, filter N on the derivative
        Kp = S.(['kp_' mot]);  Ki = S.(['ki_' mot]);  Kd = S.(['kd_' mot]);
        N = opt.N_pid;  if isfield(S, 'N_pid'), N = S.N_pid; end
        Cfb = ss(Kp + Ki/s + Kd*N*s/(s + N));
    case 'PP'       % observer + state feedback + integral of the error, r = 0:
        % u = -Kx*xhat - Ki*eI,  xhat' = A_obs*xhat + B*u + L*y,  eI' = -y
        Kx = S.(['Kx_' mot]);  Ki = S.(['Ki_' mot]);
        Aobs = S.(['A_obs_' mot]);  Bobs = S.(['B_obs_' mot]);
        B = Bobs(:,1);  L = Bobs(:,2);
        Ac = [Aobs - B*Kx, -B*Ki; zeros(1,2), 0];
        Cfb = -ss(Ac, [L; -1], [-Kx, -Ki], 0);
    otherwise
        error('Unknown controller type "%s" (use PID or PP)', type);
end
end

function d = data_kpi(S, mot, opt)
% Joint KPIs from the logged signals of one motor.
[t, r] = get_signal(S, ['r_' mot]);
if isempty(t), error('Signal r_%s not found in the run file', mot); end
y  = get_signal(S, ['y_' mot],    t);
u  = get_signal(S, ['u_' mot],    t);
us = get_signal(S, ['usat_' mot], t);
dd = get_signal(S, ['d_' mot],    t);
if isempty(y),  error('Signal y_%s not found in the run file', mot); end
if isempty(u),  u  = nan(size(t)); end
if isempty(us), us = nan(size(t)); end
if isempty(dd), dd = zeros(size(t)); end

e = r - y;
i_d = find(abs(dd) > 0, 1);
if isempty(i_d), t_dist = Inf; else, t_dist = t(i_d); end
before = t < t_dist;                       % part of the run before the disturbance
track  = before & t >= opt.t_skip;         % ... without the initial transient

d.t = t;  d.r = r;  d.y = y;  d.e = e;  d.u = u;  d.t_dist = t_dist;
k.e_max  = max(abs(e(track)));
k.e_rms  = sqrt(mean(e(track).^2));
k.u_peak = max(abs(u));
if all(isnan(u)), k.sat = 100*mean(abs(us) >= 0.999*opt.Vmax);
else,             k.sat = 100*mean(abs(u)  >= opt.Vmax);
end

% --- disturbance test: error change after d starts
[k.dist_peak, k.dist_rec, k.dist_IAE] = deal(NaN);
d.tp = [];  d.ep = [];
if isfinite(t_dist)
    pre = t >= t_dist - 0.1 & t < t_dist;
    if any(pre), e0 = mean(e(pre)); else, e0 = e(i_d); end
    post = t >= t_dist;
    d.tp = t(post) - t_dist;
    d.ep = e(post) - e0;
    k.dist_peak = max(abs(d.ep));
    thr  = max(0.02*k.dist_peak, opt.enc_res);       % never below one count
    last = find(abs(d.ep) > thr, 1, 'last');
    if isempty(last),            k.dist_rec = 0;
    elseif last == numel(d.ep),  k.dist_rec = NaN;   % not recovered by the end
    else,                        k.dist_rec = d.tp(last+1);
    end
    k.dist_IAE = trapz(d.tp, abs(d.ep));
end

% --- step test: r changes in ONE interval only (works also with a rate limiter)
[k.rise, k.settling, k.overshoot] = deal(NaN);
d.ts = [];  d.z = [];
rb  = r(before);
dr  = diff(rb);
mov = find(abs(dr) > 1e-9*max(1, max(abs(rb))));
is_step = ~isempty(mov) && mov(end) - mov(1) + 1 == numel(mov) ...   % one interval
          && mov(end) < numel(rb) - 1 ...                             % then constant
          && (all(dr(mov) > 0) || all(dr(mov) < 0)) ...               % monotonic
          && abs(rb(mov(end)+1) - rb(mov(1))) > 0.1;                  % > 0.1 deg
if is_step
    j  = mov(1);                                     % start of the change
    r0 = rb(j);  r1 = rb(mov(end) + 1);
    w  = j:numel(rb);
    d.ts = t(w) - t(j);
    d.z  = (y(w) - r0)/(r1 - r0);                    % normalized: 0 -> 1
    si = stepinfo(d.z, d.ts, 1);
    k.rise = si.RiseTime;  k.settling = si.SettlingTime;  k.overshoot = si.Overshoot;
end
d.kpi = k;
end

function p = tip_kpi(S, t_dist, t_skip)
% Tip errors from xc_real, yc_real (achieved) and xc_des, yc_des (desired).
[t, xr] = get_signal(S, 'xc_real');
yr = get_signal(S, 'yc_real', t);
xd = get_signal(S, 'xc_des',  t);
yd = get_signal(S, 'yc_des',  t);
p = struct('t', t, 'P', [], 'Pd', [], 'e_track', [], 'e_contour', [], ...
           'before', [], 'kpi', []);
k.contour_max = NaN;  k.contour_rms = NaN;  k.track_max = NaN;
if isempty(t) || isempty(yr) || isempty(xd) || isempty(yd)
    warning('Tip signals (xc_real, yc_real, xc_des, yc_des) missing in a run');
    p.kpi = k;  return
end
before = t >= t_skip & t < t_dist;        % without initial transient and disturbance
p.P  = [xr(:)'; yr(:)'];
p.Pd = [xd(:)'; yd(:)'];
p.e_track   = vecnorm(p.P - p.Pd);
p.e_contour = nan(size(p.e_track));
p.e_contour(before) = dist_to_path(p.P(:,before), p.Pd(:,before));
k.contour_max = 1e3*max(p.e_contour(before));
k.contour_rms = 1e3*sqrt(mean(p.e_contour(before).^2));
k.track_max   = 1e3*max(p.e_track(before));
p.before = before;
p.kpi = k;
end

function d = dist_to_path(P, Q)
% Minimum distance of each point P(:,n) from the polyline through Q.
A  = Q(:,1:end-1);
AB = Q(:,2:end) - A;
L2 = max(sum(AB.^2, 1), eps);
d  = zeros(1, size(P,2));
for n = 1:size(P,2)
    AP = P(:,n) - A;
    s  = min(max(sum(AP.*AB, 1)./L2, 0), 1);
    d(n) = min(vecnorm(P(:,n) - (A + AB.*s)));
end
end

function [t, x] = get_signal(S, name, t_ref)
% Reads a logged signal from S.out (SimulationOutput) or from S directly.
% Accepts Timeseries, Structure With Time or plain arrays (with S.Ts or 2 ms).
% With t_ref it returns only the values interpolated on t_ref.
names = {name};                     % accepted aliases
if strcmp(name, 'xc_real'), names = {'xc_real', 'xc_sim'}; end
if strcmp(name, 'yc_real'), names = {'yc_real', 'yc_sim'}; end
v = [];
for c = 1:numel(names)
    v = find_var(S, names{c});
    if ~isempty(v), break; end
end
t = [];  x = [];
if isempty(v)
    if nargin > 2, t = []; end
    return
end
if isa(v, 'timeseries')
    t = v.Time;  x = squeeze(v.Data);
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;  x = squeeze(v.signals.values);
else
    x = squeeze(v);
    if isfield(S, 'Ts'), Ts = S.Ts; else, Ts = 2e-3; end
    t = (0:numel(x)-1)'*Ts;
end
t = t(:);  x = double(x(:));
[t, iu] = unique(t, 'last');    % variable-step solvers can repeat time stamps
x = x(iu);
if nargin > 2
    x = interp1(t, x, t_ref, 'linear', 'extrap');
    t = x;                      % single output: the values on t_ref
end
end

function f = find_run(name, data_dir)
% full path, or relative to data/ ('sim/...', 'lab/...'), or name only
cand = {name, fullfile(data_dir, name), fullfile(data_dir, 'sim', name), ...
        fullfile(data_dir, 'lab', name)};
ok = cand(cellfun(@isfile, cand));
if isempty(ok), error('Run file "%s" not found in data/, data/sim, data/lab', name); end
f = ok{1};
if numel(unique(ok)) > 1
    warning('"%s" exists in more than one folder: using %s', name, f);
end
end

function v = find_var(S, name)
% name, or name_<tag> as written by run_and_save, in S.out or in S
v = [];
if isfield(S, 'out') && isa(S.out, 'Simulink.SimulationOutput') && any(strcmp(S.out.who, name))
    v = S.out.get(name);  return
end
if isfield(S, name), v = S.(name); return, end
if isfield(S, 'run_tag') && isfield(S, [name '_' S.run_tag])
    v = S.([name '_' S.run_tag]);  return
end
fn = fieldnames(S);
m  = fn(~cellfun(@isempty, regexp(fn, ['^' name '_.+_\d{8}_\d{6}$'], 'once')));
if numel(m) == 1, v = S.(m{1}); end
if numel(m) > 1, error('Several runs of %s in the same file: %s', name, strjoin(m', ', ')); end
end

function print_table(label, rows, K, names)
M = nan(size(rows,1), numel(K));
for j = 1:numel(K)
    for r = 1:size(rows,1)
        M(r,j) = K(j).(rows{r,1});
    end
end
tbl = array2table(round(M, 4), 'RowNames', rows(:,2), ...
                  'VariableNames', matlab.lang.makeValidName(names));
fprintf('\n================ %s ================\n', label);
disp(tbl);
end

function plot_robustness(m, names, col, label)
nR = numel(m);
light = @(c) 0.35*c + 0.65;
th = linspace(0, 2*pi, 200);
figure('Name', ['Robustness - ' label], 'Color', 'w');
tl = tiledlayout(2, 3, 'TileSpacing', 'compact');
title(tl, ['Robustness (model) - ' label]);

nexttile([2 1]); hold on; grid on; box on; axis equal
for j = 1:nR
    for b = 1:numel(m(j).bundle)
        plot(real(m(j).bundle{b}), imag(m(j).bundle{b}), 'Color', light(col(j,:)), 'LineWidth', 0.5);
    end
end
h = gobjects(nR, 1);
for j = 1:nR
    Lw = m(j).Lw;  Ms = m(j).kpi.Ms;
    h(j) = plot(real(Lw), imag(Lw), 'Color', col(j,:), 'LineWidth', 1.5);
    plot(real(Lw), -imag(Lw), ':', 'Color', col(j,:));
    plot(-1 + cos(th)/Ms, sin(th)/Ms, '--', 'Color', col(j,:));
end
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
xlim([-3 1]); ylim([-2.5 2.5]); xlabel('Re L'); ylabel('Im L');
title({'Nyquist of L', 'dashed: circle of radius 1/M_s around -1; thin: k,\tau varied'});
legend(h, names, 'Location', 'southoutside');

nexttile; hold on; grid on; box on
for j = 1:nR, plot(m(j).w, 20*log10(abs(m(j).Lw)), 'Color', col(j,:), 'LineWidth', 1.2); end
set(gca, 'XScale', 'log'); yline(0, 'k:'); ylabel('|L| [dB]'); title('Bode of L - magnitude');

nexttile; hold on; grid on; box on
for j = 1:nR
    plot(m(j).w, unwrap(angle(m(j).Lw))*180/pi, 'Color', col(j,:), 'LineWidth', 1.2);
    xline(m(j).kpi.wc, ':', 'Color', col(j,:));
end
set(gca, 'XScale', 'log'); yline(-180, 'k:');
xlabel('\omega [rad/s]'); ylabel('\angle L [deg]'); title('Bode of L - phase (dotted: \omega_c)');

nexttile; hold on; grid on; box on
for j = 1:nR
    plot(m(j).w, 20*log10(abs(m(j).Sw)), '-',  'Color', col(j,:), 'LineWidth', 1.2);
    plot(m(j).w, 20*log10(abs(m(j).Tw)), '--', 'Color', col(j,:));
    [Ms, iM] = max(abs(m(j).Sw));
    plot(m(j).w(iM), 20*log10(Ms), 'o', 'Color', col(j,:), 'MarkerFaceColor', col(j,:));
end
set(gca, 'XScale', 'log'); yline(0, 'k:');
xlabel('\omega [rad/s]'); ylabel('[dB]'); title('|S| solid, |T| dashed (dot = M_s)');

nexttile; hold on; grid on; box on
lim = 0;
for j = 1:nR
    L = m(j).L;
    R = rlocus(L, [0, logspace(-3, 2, 600)]);
    plot(real(R(:)), imag(R(:)), '.', 'Color', light(col(j,:)), 'MarkerSize', 3);
    pol = pole(L);  zer = tzero(L);  pcl = pole(feedback(L, 1));
    plot(real(pol), imag(pol), 'x', 'Color', col(j,:), 'MarkerSize', 9, 'LineWidth', 1.5);
    plot(real(zer), imag(zer), 'o', 'Color', col(j,:), 'MarkerSize', 8, 'LineWidth', 1.2);
    plot(real(pcl), imag(pcl), 's', 'Color', col(j,:), 'MarkerFaceColor', col(j,:));
    lim = max(lim, max(abs(pcl)));
end
xline(0, 'k:'); xlim([-1.3*lim, 0.3*lim]); ylim([-1.3*lim, 1.3*lim]);
xlabel('Re'); ylabel('Im'); title('Root locus of L (x poles, o zeros, filled squares: current CL poles)');
end

function plot_joints(dat, names, col, motors, opt)
nR = size(dat, 2);
figure('Name', 'Data - joints', 'Color', 'w');
tl = tiledlayout(3, 2, 'TileSpacing', 'compact');
title(tl, 'Joints (logged data)');
for row = 1:3
    for i = 1:2
        nexttile; hold on; grid on; box on
        h = gobjects(nR, 1);
        for j = 1:nR
            d = dat(i,j);
            switch row
                case 1
                    h(j) = plot(d.t, d.y, 'Color', col(j,:), 'LineWidth', 1.2);
                    plot(d.t, d.r, '--', 'Color', col(j,:));
                    ylabel('y, r (dashed) [deg]');
                case 2
                    h(j) = plot(d.t, d.e, 'Color', col(j,:), 'LineWidth', 1.0);
                    ylabel('e = r - y [deg]');
                case 3
                    h(j) = plot(d.t, d.u, 'Color', col(j,:), 'LineWidth', 1.0);
                    ylabel('u [V]');
            end
        end
        if row == 1, title(['motor ' motors{i}]); legend(h, names, 'Location', 'best'); end
        if row == 3, yline([-opt.Vmax opt.Vmax], 'r--'); xlabel('t [s]'); end
    end
end
end

function plot_tests(dat, names, col, motors)
has_d = any(arrayfun(@(x) ~isempty(x.tp), dat(:)));
has_s = any(arrayfun(@(x) ~isempty(x.ts), dat(:)));
if ~has_d && ~has_s, return; end
nR = size(dat, 2);
figure('Name', 'Data - disturbance and step', 'Color', 'w');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact');
title(tl, 'Disturbance and step tests (logged data)');
for i = 1:2
    nexttile; hold on; grid on; box on
    for j = 1:nR
        if ~isempty(dat(i,j).tp), plot(dat(i,j).tp, dat(i,j).ep, 'Color', col(j,:), 'LineWidth', 1.2); end
    end
    yline(0, 'k:'); xlabel('t - t_d [s]'); ylabel('error change [deg]');
    title(['Torque disturbance - motor ' motors{i}]);
end
for i = 1:2
    nexttile; hold on; grid on; box on
    h = gobjects(0);  lab = {};
    for j = 1:nR
        if ~isempty(dat(i,j).ts)
            h(end+1) = plot(dat(i,j).ts, dat(i,j).z, 'Color', col(j,:), 'LineWidth', 1.2); %#ok<AGROW>
            lab{end+1} = names{j}; %#ok<AGROW>
        end
    end
    yline([0.98 1.02], 'k:'); xlabel('t - t_{step} [s]'); ylabel('y normalized [-]');
    title(['Reference step - motor ' motors{i}]);
    if ~isempty(h), legend(h, lab, 'Location', 'southeast'); end
end
end

function plot_tip(tip, names, col)
ok = arrayfun(@(x) ~isempty(x.P), tip);
if ~any(ok), return; end
nR = numel(tip);
ref = tip(find(ok, 1));
Pd  = ref.Pd(:, ref.before);
v   = diff(Pd, 1, 2);
v   = v(:, vecnorm(v) > 1e-9);                     % drop repeated points
ang = abs(atan2(v(1,1:end-1).*v(2,2:end) - v(2,1:end-1).*v(1,2:end), ...
                sum(v(:,1:end-1).*v(:,2:end), 1)));
[~, ic] = max(ang);
P_unique = Pd(:, [true, vecnorm(diff(Pd,1,2)) > 1e-9]);
pc = P_unique(:, min(ic+1, size(P_unique,2)));
zw = 3e-3;

figure('Name', 'Tip (x, y)', 'Color', 'w');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact');
title(tl, 'Tip (logged data, after t_{skip} and before the disturbance)');
for z = 1:2
    nexttile; hold on; grid on; box on; axis equal
    h0 = plot(Pd(1,:), Pd(2,:), 'k--', 'LineWidth', 1.2);
    h = gobjects(0);  lab = {};
    for j = find(ok)
        b = tip(j).before;
        h(end+1) = plot(tip(j).P(1,b), tip(j).P(2,b), 'Color', col(j,:), 'LineWidth', 1.2); %#ok<AGROW>
        lab{end+1} = names{j}; %#ok<AGROW>
    end
    xlabel('x [m]'); ylabel('y [m]');
    if z == 1
        title('Path'); legend([h0, h], [{'desired'}, lab], 'Location', 'best');
    else
        xlim(pc(1) + [-zw zw]); ylim(pc(2) + [-zw zw]);
        title(sprintf('Zoom on the sharpest corner (\\pm%g mm)', 1e3*zw));
    end
end
nexttile; hold on; grid on; box on
for j = find(ok), plot(tip(j).t, 1e3*tip(j).e_contour, 'Color', col(j,:), 'LineWidth', 1.2); end
xlabel('t [s]'); ylabel('[mm]'); title('Contour error (distance from the path)');
nexttile; hold on; grid on; box on
for j = find(ok)
    b = tip(j).before;
    plot(tip(j).t(b), 1e3*tip(j).e_track(b), 'Color', col(j,:), 'LineWidth', 1.2);
end
xlabel('t [s]'); ylabel('[mm]'); title('Tracking error (distance from the point at time t)');
end