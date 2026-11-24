%% PP_TUNE_ANALYZE - read a pp_tune report, check the requirements, rank the designs
%
% WHAT IT DOES
%   1. Reads a report written by pp_tune.m (data/sim/pp_tune_<date>_<time>.txt;
%      older observer_study_*.txt reports are read too).
%   2. Accepts a design only if every requirement is met, up to the tolerance of section 1
%      (one per requirement, as a fraction of the limit: 0 = met exactly, < 0 = stricter).
%   3. Loads the run file of each design and computes, after t_skip:
%        joint error  RMS of reference - angle, motor 3165 and 3166            [deg]
%        tip tracking distance between actual and desired tip, same instant  [mm]
%        contour      distance of the actual tip from the desired path       [mm]
%        HF voltage   RMS of the voltage above 20 Hz (vibrations)            [V]
%   4. Prints the accepted designs (best first, with PM, Ms, wc of the worst motor), the
%      rejected ones, and checks the best design against the targets tgt of section 1.
%   5. FINAL REPORT (also saved as data/sim/analysis_<report name>.txt):
%        a) best design for each observer option (rows of obs_list in pp_tune.m)
%        b) best design for each requirement and each performance metric
%        c) OVERALL BEST with coherent margins: accepted, PM / Ms / wc targets met and
%           the two motors with the same crossover (axis matching, see tgt.wc_match)
%
% HOW THE RANKING WORKS
%   score = mean of (metric / best value among the accepted designs) over joint error,
%   tip tracking RMS, contour RMS and contour max. 1.00 = best on everything.
%   "use" = how much of the Ms and chatter limits the design uses (100 % = at the limit):
%   the best designs are the ones that push close to the limits with the lowest score.

clear; clc
this     = fileparts(mfilename('fullpath'));                       % .../pp_design
data_sim = fullfile(this, '..', '..', '..', 'data', 'sim');        % .../data/sim

%% 1. Settings
study_file = '';          % report name in data/sim; '' = the newest pp_tune_*.txt
% Tolerances: how much a design may MISS each requirement, as a fraction of its limit
% (0 = met exactly, 0.05 = 5 %). For "<=" limits the value may go above the limit, for ">="
% limits below it. Example: PM >= 45 with tol.PM = 0.05 accepts PM down to 42.75 deg.
% NEGATIVE values make the check STRICTER (the design must beat the limit by that margin):
% tol.PM = -0.1 requires PM >= 49.5 deg, tol.Ms = -0.05 requires Ms <= 0.95*limit.
tol.peak    = -0.2;        % peak voltage   [V]     (<= limit)
tol.PM      = -0.05;       % phase margin   [deg]   (>= limit)
tol.Ms      = 0;           % Ms             [-]     (<= limit)
tol.chatter = 0;           % chatter        [V]     (<= limit)
tol.slow    = 0;           % slowest pole   [rad/s] (>= limit)
tol.wc      = 0.05;        % crossover wc   [rad/s] (<= limit)

% margin targets of the final choice (worst motor), used by the check of the best design
% and by the OVERALL BEST of the final report (can be stricter than the limits of pp_tune)
tgt.PM_min   = 45;        % [deg]   phase margin
tgt.Ms_max   = 1.6;       % [-]     peak of |S|: GM >= Ms/(Ms-1), PM >= 2 asin(1/(2 Ms))
tgt.wc       = [20, 40];  % [rad/s] crossover window: at least the PID bandwidth (20 rad/s),
                          %         at most the 40 rad/s limit of pole_place.m
tgt.wc_match = 0.15;      % [-]     max wc mismatch between the two motors, (max-min)/mean:
                          %         same bandwidth on both axes = no shape distortion
t_skip = 0;               % [s]     analysis starts here (skip the move from home to the path)
f_hf   = 20;              % [Hz]    HF voltage = content above this frequency

%% 2. Read the report
if isempty(study_file)
    d = [dir(fullfile(data_sim, 'pp_tune_*.txt')); dir(fullfile(data_sim, 'observer_study_*.txt'))];
    [~, newest] = max([d.datenum]);
    study_file = d(newest).name;
end
txt = fileread(fullfile(data_sim, study_file));
% save everything printed from here on (Command Window copy)
[~, rep_name] = fileparts(study_file);
out_file = fullfile(data_sim, ['analysis_' rep_name '.txt']);
if isfile(out_file), delete(out_file); end
diary(out_file);
txt_lines = splitlines(txt);   % (not "lines": that would hide the MATLAB function lines)
fprintf('Report: %s\n', study_file);
fprintf(['Tolerances (%% of the limit): peak V %g, PM %g, Ms %g, chatter %g, slowest pole %g, ' ...
         'wc %g\n\n'], 100*[tol.peak, tol.PM, tol.Ms, tol.chatter, tol.slow, tol.wc]);

D = struct('label', {}, 'file', {}, 'names', {}, 'sgn', {}, 'lim', {}, 'val', {}, 'wc', {});
for n = 1:numel(txt_lines)
    s = txt_lines{n};
    tok = regexp(s, '^==== \d+\s+(.+?)\s{2,}\(', 'tokens', 'once');      % start of a design
    if ~isempty(tok)
        D(end+1).label = strtrim(tok{1}); %#ok<SAGROW>
        D(end).names = {};  D(end).sgn = {};  D(end).lim = [];  D(end).val = zeros(0, 2);
        D(end).wc = [NaN, NaN];                                          % old reports: no wc
        continue
    end
    if isempty(D), continue, end
    % older reports: crossover wc of motor 3165 and 3166 on an "(info)" line
    tok = regexp(s, '^\s+crossover wc \[rad/s\]\s+\(info\)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', 'tokens', 'once');
    if ~isempty(tok)
        D(end).wc = [str2double(tok{1}), str2double(tok{2})];
        continue
    end
    % requirement line: name, <= or >=, limit, value motor 3165, value motor 3166 (* optional)
    tok = regexp(s, '^\s+(.+?)\s+(<=|>=)\s+([\d.eE+-]+)\s+([\d.eE+-]+)\*?\s+([\d.eE+-]+)\*?', 'tokens', 'once');
    if ~isempty(tok)
        D(end).names{end+1} = strtrim(tok{1});
        D(end).sgn{end+1}   = tok{2};
        D(end).lim(end+1)   = str2double(tok{3});
        D(end).val(end+1, :) = [str2double(tok{4}), str2double(tok{5})];
        continue
    end
    tok = regexp(s, 'run file\(s\):\s*(\S+\.mat)', 'tokens', 'once');      % first run file
    if ~isempty(tok)
        D(end).file = tok{1};
    end
end

%% 3. Check the requirements with the tolerances
nD = numel(D);
ok = true(nD, 1);  why = cell(nD, 1);  useMs = nan(nD, 1);  useCh = nan(nD, 1);
PMw = nan(nD, 1);  Msw = nan(nD, 1);  wcw = nan(nD, 2);    % worst PM, worst Ms, wc range
for i = 1:nD
    bad = {};
    wcw(i, :) = [min(D(i).wc), max(D(i).wc)];               % slower and faster motor
    for k = 1:numel(D(i).names)
        v = D(i).val(k, :);  L = D(i).lim(k);
        if strcmp(D(i).sgn{k}, '<='), excess = max(v)/L - 1;     % worst motor
        else,                         excess = 1 - min(v)/L;
        end
        t = req_tol(D(i).names{k}, tol);                         % tolerance of this requirement
        if startsWith(D(i).names{k}, 'Ms'),      useMs(i) = max(v)/L; Msw(i) = max(v); end
        if startsWith(D(i).names{k}, 'chatter'), useCh(i) = max(v)/L; end
        if startsWith(D(i).names{k}, 'phase'),   PMw(i) = min(v); end
        if startsWith(D(i).names{k}, 'crossover'), wcw(i, :) = [min(v), max(v)]; end   % new reports
        if excess > t + 1e-9
            bad{end+1} = sprintf('%s %+.1f%%', strtok(D(i).names{k}, ' '), 100*excess); %#ok<AGROW>
        end
    end
    ok(i) = isempty(bad);
    why{i} = strjoin(bad, ', ');
end

%% 4. Metrics of every run
M = nan(nD, 7);   % joint 3165, joint 3166, tip RMS, tip max, contour RMS, contour max, HF voltage
for i = 1:nD
    f = fullfile(data_sim, D(i).file);
    if isempty(D(i).file) || ~isfile(f)
        warning('Run file of "%s" not found: skipped.', D(i).label);
        continue
    end
    M(i, :) = run_metrics(f, t_skip, f_hf);
    fprintf('read %2d/%d  %s\n', i, nD, D(i).label);
end

%% 5. Ranking of the accepted designs and printout
acc = find(ok & ~isnan(M(:, 1)));
cols = [mean(M(:, 1:2), 2), M(:, 3), M(:, 5), M(:, 6)];      % metrics used in the score
score = nan(nD, 1);
if ~isempty(acc)                                             % no accepted design: no ranking
    best = min(cols(acc, :), [], 1);                         % 1 x 4, best of each metric
    score = mean(cols./best, 2);
    [~, idx] = sort(score(acc));
    acc = acc(idx);
else
    fprintf('\nNo design meets every requirement with these tolerances (section 1).\n');
    fprintf('See the REJECTED list: the %% is how much each requirement is exceeded.\n');
    fprintf('Options: lower wn or ifac (Ms, chatter), slower observer (chatter), or raise tol.\n');
end

fprintf('\n=========== ACCEPTED (requirements met within the tolerances), best first ===========\n');
fprintf('%-28s %5s %6s %6s | %5s %5s %11s | %13s %13s %13s %6s\n', 'design', 'score', 'Ms', 'chat.', ...
        'PM', 'Ms', 'wc [rad/s]', 'joint [deg]', 'tip [mm]', 'contour [mm]', 'HF [V]');
fprintf('%-28s %5s %6s %6s | %5s %5s %11s | %13s %13s %13s %6s\n', '', '', 'use', 'use', ...
        'worst', 'worst', 'min / max', '3165 / 3166', 'RMS / max', 'RMS / max', '');
for i = acc'
    fprintf('%-28s %5.2f %5.0f%% %5.0f%% | %5.1f %5.2f %5.1f/%5.1f | %6.3f/%6.3f %6.3f/%6.3f %6.3f/%6.3f %6.3f\n', ...
        D(i).label, score(i), 100*useMs(i), 100*useCh(i), PMw(i), Msw(i), wcw(i, :), M(i, 1:7));
end

rej = find(~ok);
if ~isempty(rej)
    fprintf('\n=========== REJECTED (excess over the limit; PM, Ms, wc of the worst motor) ===========\n');
    for i = rej'
        fprintf('%-28s PM %5.1f  Ms %4.2f  wc %5.1f-%5.1f | %s\n', D(i).label, PMw(i), Msw(i), ...
                wcw(i, :), why{i});
    end
end

if ~isempty(acc)
    i = acc(1);
    fprintf(['\nBest: %s  (score %.2f; uses %.0f %% of the Ms limit and %.0f %% of the ' ...
             'chatter limit)\n'], D(i).label, score(i), 100*useMs(i), 100*useCh(i));
    fprintf('Score: 1.00 = best of the accepted designs on joint error, tip RMS, contour RMS and max.\n');

    % frequency-domain check of the best design (worst motor)
    fprintf('\nCheck of the best design (worst motor):\n');
    fprintf('  PM = %5.1f deg     target >= %g        %s\n', PMw(i), tgt.PM_min, okstr(PMw(i) >= tgt.PM_min));
    fprintf('  Ms = %5.2f         target <= %g       %s\n', Msw(i), tgt.Ms_max, okstr(Msw(i) <= tgt.Ms_max));
    fprintf('  wc = %4.1f-%4.1f rad/s  window [%g, %g]  %s\n', wcw(i, :), tgt.wc, ...
            okstr(wcw(i, 1) >= tgt.wc(1) && wcw(i, 2) <= tgt.wc(2)));
    if any(isnan(wcw(i, :)))
        fprintf('  (no wc in this report: rerun pp_tune.m)\n');
    end

    % best accepted design that also meets the three targets (if different)
    good = acc(PMw(acc) >= tgt.PM_min & Msw(acc) <= tgt.Ms_max & ...
               wcw(acc, 1) >= tgt.wc(1) & wcw(acc, 2) <= tgt.wc(2));
    if isempty(good)
        fprintf('No accepted design meets PM, Ms and the wc window together: lower wn or ifac.\n');
    elseif good(1) ~= i
        g = good(1);
        fprintf(['Best design inside the PM / Ms / wc targets: %s  (score %.2f, PM %.1f deg, ' ...
                 'Ms %.2f, wc %.1f-%.1f rad/s)\n'], D(g).label, score(g), PMw(g), Msw(g), wcw(g, :));
    end
end

%% 6. Final report: per observer, per requirement, overall best
fprintf('\n\n#################### FINAL REPORT ####################\n');
if isempty(acc)
    fprintf('No accepted design: nothing to select (see the REJECTED list).\n');
else
    % ---- a) best design for each observer option ----
    obs = cell(nD, 1);                                   % observer tag of each design, e.g. '2.5'
    for i = 1:nD
        t = regexp(D(i).label, 'obs(\S+)$', 'tokens', 'once');
        if isempty(t), obs{i} = '?'; else, obs{i} = t{1}; end
    end
    fprintf('\na) BEST FOR EACH OBSERVER OPTION (score among the accepted designs)\n');
    fprintf('   %-8s %-28s %5s | %5s %5s %11s | %s\n', 'observer', 'best design', 'score', ...
            'PM', 'Ms', 'wc [rad/s]', 'accepted');
    for o = unique(obs)'
        in = acc(strcmp(obs(acc), o{1}));                % accepted, this observer, best first
        n_all = nnz(strcmp(obs, o{1}));
        if isempty(in)
            fprintf('   %-8s %-28s\n', [o{1} 'x'], '(none accepted)');
            continue
        end
        i = in(1);
        fprintf('   %-8s %-28s %5.2f | %5.1f %5.2f %5.1f/%5.1f | %d of %d\n', [o{1} 'x'], ...
                D(i).label, score(i), PMw(i), Msw(i), wcw(i, :), numel(in), n_all);
    end

    % ---- b) best design for each requirement and each metric ----
    % requirement: the design with the most margin from its limit (worst motor), except wc,
    % where the best is the FASTEST loop still inside the limit (more bandwidth = less error).
    fprintf('\nb) BEST FOR EACH REQUIREMENT (worst motor) AND EACH METRIC\n');
    names = D(acc(1)).names;
    for k = 1:numel(names)
        v = nan(numel(acc), 1);
        for a = 1:numel(acc)
            kk = find(strcmp(D(acc(a)).names, names{k}), 1);
            if isempty(kk), continue, end
            if strcmp(D(acc(a)).sgn{kk}, '<='), v(a) = max(D(acc(a)).val(kk, :));
            else,                               v(a) = min(D(acc(a)).val(kk, :));
            end
        end
        hi = strcmp(D(acc(1)).sgn{k}, '>=') || startsWith(names{k}, 'crossover');
        if hi, [vb, a] = max(v); what = 'highest'; else, [vb, a] = min(v); what = 'lowest'; end
        fprintf('   %-24s %-8s %9.3f   %s\n', names{k}, what, vb, D(acc(a)).label);
    end
    met = {'joint error RMS [deg]', 'tip tracking RMS [mm]', 'tip tracking max [mm]', ...
           'contour RMS [mm]', 'contour max [mm]', 'HF voltage [V]'};
    Mm  = [mean(M(:, 1:2), 2), M(:, 3:7)];               % joint error = mean of the two motors
    for k = 1:numel(met)
        [vb, a] = min(Mm(acc, k));
        fprintf('   %-24s %-8s %9.3f   %s\n', met{k}, 'lowest', vb, D(acc(a)).label);
    end

    % ---- c) overall best with coherent margins ----
    % coherent = accepted (every requirement met) AND PM, Ms inside the stricter targets AND
    % wc inside the window AND the same wc on both motors (axis matching). Ranked by score.
    mis  = (wcw(:, 2) - wcw(:, 1))./mean(wcw, 2);        % wc mismatch between the motors
    coh  = acc(PMw(acc) >= tgt.PM_min & Msw(acc) <= tgt.Ms_max & wcw(acc, 1) >= tgt.wc(1) & ...
               wcw(acc, 2) <= tgt.wc(2) & mis(acc) <= tgt.wc_match);
    fprintf(['\nc) OVERALL BEST WITH COHERENT MARGINS  (PM >= %g deg, Ms <= %g, wc in [%g, %g] ' ...
             'rad/s, motor mismatch <= %g %%)\n'], tgt.PM_min, tgt.Ms_max, tgt.wc, 100*tgt.wc_match);
    if isempty(coh)
        fprintf('   none: no accepted design meets all the margin targets. Best accepted is %s.\n', ...
                D(acc(1)).label);
        fprintf('   Try lower wn or ifac (more PM, lower Ms) or relax tgt in section 1.\n');
    else
        fprintf('   %-4s %-28s %5s | %5s %5s %11s %6s | %13s %13s\n', 'rank', 'design', 'score', ...
                'PM', 'Ms', 'wc [rad/s]', 'mism.', 'tip RMS [mm]', 'contour [mm]');
        for r = 1:min(3, numel(coh))                    % podium
            i = coh(r);
            fprintf('   %-4d %-28s %5.2f | %5.1f %5.2f %5.1f/%5.1f %5.1f%% | %13.3f %6.3f/%6.3f\n', ...
                    r, D(i).label, score(i), PMw(i), Msw(i), wcw(i, :), 100*mis(i), M(i, 3), M(i, 5:6));
        end
        i = coh(1);
        fprintf('\n   >>> OVERALL BEST: %s  -> copy wn, zeta, lambda = ifac*zeta*wn and the observer\n', ...
                D(i).label);
        fprintf('       factor into pole_place.m (lambda in rad/s, see the ==== line of the report).\n');
    end
end
fprintf('\nReport saved in %s\n', out_file);
diary off


%% ---------------- local functions ----------------
function t = req_tol(name, tol)
% Tolerance of a requirement, found from the start of its name in the report.
if     startsWith(name, 'peak'),      t = tol.peak;
elseif startsWith(name, 'phase'),     t = tol.PM;
elseif startsWith(name, 'Ms'),        t = tol.Ms;
elseif startsWith(name, 'chatter'),   t = tol.chatter;
elseif startsWith(name, 'slowest'),   t = tol.slow;
elseif startsWith(name, 'crossover'), t = tol.wc;
else,                                 t = 0;          % unknown requirement: met exactly
end
end

function s = okstr(b)
% 'ok' / 'NOT met' for the check printout
if b, s = 'ok'; else, s = 'NOT met'; end
end

function m = run_metrics(file, t_skip, f_hf)
% Joint error, tip tracking, contour error and HF voltage of one saved run.
S = load(file);
r1 = get_ts(S, 'r_3165');   y1 = get_ts(S, 'y_3165');   u1 = get_ts(S, 'u_3165');
r2 = get_ts(S, 'r_3166');   y2 = get_ts(S, 'y_3166');   u2 = get_ts(S, 'u_3166');
[xd, t] = get_ts(S, 'xc_des');   yd = get_ts(S, 'yc_des');
xs = get_ts(S, 'xc_sim');        ys = get_ts(S, 'yc_sim');
k  = t >= t_skip;                                   % periodic regime only
Ts = median(diff(t));
m  = nan(1, 7);                                     % NaN = not computable (always 7 values)
if nnz(k) < 4
    warning('%s: run of %.2f s has no samples after t_skip = %g s: lower t_skip.', ...
            file, t(end), t_skip);
    return
end

e1 = r1(k) - y1(k);  e2 = r2(k) - y2(k);
tip = 1000*hypot(xs(k) - xd(k), ys(k) - yd(k));     % [mm]
con = 1000*dist_to_path(xs(k), ys(k), xd(k), yd(k));% [mm]
hf  = max(hf_rms(u1(k), Ts, f_hf), hf_rms(u2(k), Ts, f_hf));

rms_ = @(x) sqrt(mean(x.^2));                      % RMS (no toolbox needed)
m = [rms_(e1), rms_(e2), rms_(tip), max(tip), rms_(con), max(con), hf];
end

function [x, t] = get_ts(S, prefix)
% The signal whose variable name starts with prefix_ (e.g. r_3165_Model_pp_obs_<tag>).
f = fieldnames(S);
name = f(startsWith(f, [prefix '_']));
ts = S.(name{1});                                   % timeseries
x = squeeze(ts.Data);
t = ts.Time;
end

function d = dist_to_path(xs, ys, xd, yd)
% Distance of each actual tip point from the nearest point of the desired path.
% Computed in blocks of 500 points to keep the matrices small.
d = zeros(size(xs));
for a = 1:500:numel(xs)
    b = min(a + 499, numel(xs));
    D2 = (xs(a:b) - xd.').^2 + (ys(a:b) - yd.').^2;   % block x path points
    d(a:b) = sqrt(min(D2, [], 2));
end
end

function v = hf_rms(u, Ts, f_hf)
% RMS of the part of u above f_hf, from the FFT (Parseval): no toolbox needed.
u = u - mean(u);
N = numel(u);
U = fft(u);
f = (0:N-1)'/(N*Ts);
f = min(f, 1/Ts - f);                               % two-sided: |frequency|
v = sqrt(sum(abs(U(f > f_hf)).^2))/N;
end
