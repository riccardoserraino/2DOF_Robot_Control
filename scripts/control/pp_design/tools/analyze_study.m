%% ANALYZE STUDY - read an observer_study report, check it with tolerances, rank the designs
%
% WHAT IT DOES
%   1. Reads a report written by observer_study.m (data/sim/observer_study_<date>_<time>.txt).
%   2. Accepts a design if every requirement is met, allowing a small excess on the two most
%      critical ones: Ms up to +3 %, chatter up to +6 % (set in section 1).
%   3. Loads the run file of each design and computes, after t_skip (periodic regime):
%        joint error  RMS of reference - angle, motor 3165 and 3166            [deg]
%        tip tracking distance between actual and desired tip, same instant  [mm]
%        contour      distance of the actual tip from the desired path       [mm]
%        HF voltage   RMS of the voltage above 20 Hz (vibrations)            [V]
%   4. Prints in the Command Window the accepted designs, best first, and the rejected ones.
%
% HOW THE RANKING WORKS
%   score = mean of (metric / best value among the accepted designs) over joint error,
%   tip tracking RMS, contour RMS and contour max. 1.00 = best on everything.
%   "use" = how much of the Ms and chatter limits the design uses (100 % = at the limit):
%   the best designs are the ones that push close to the limits with the lowest score.

clear; clc
this     = fileparts(mfilename('fullpath'));                       % .../pp_design/tools
data_sim = fullfile(this, '..', '..', '..', '..', 'data', 'sim');  % .../data/sim

%% 1. Settings
study_file = '';          % report name in data/sim; '' = the newest observer_study_*.txt
tol.Ms      = 0.03;       % Ms may exceed its limit by 3 %
tol.chatter = 0.06;       % chatter may exceed its limit by 6 %
                          % every other requirement must be met exactly
t_skip = 0;               % [s] start of the periodic regime
f_hf   = 20;              % [Hz] HF voltage = content above this frequency

%% 2. Read the report
if isempty(study_file)
    d = dir(fullfile(data_sim, 'observer_study_*.txt'));
    [~, newest] = max([d.datenum]);
    study_file = d(newest).name;
end
txt = fileread(fullfile(data_sim, study_file));
txt_lines = splitlines(txt);   % (not "lines": that would hide the MATLAB function lines)
fprintf('Report: %s\n', study_file);
fprintf('Tolerances: Ms +%g %%, chatter +%g %%, others 0 %%\n\n', 100*tol.Ms, 100*tol.chatter);

D = struct('label', {}, 'file', {}, 'names', {}, 'sgn', {}, 'lim', {}, 'val', {});
for n = 1:numel(txt_lines)
    s = txt_lines{n};
    tok = regexp(s, '^==== \d+\s+(.+?)\s{2,}\(', 'tokens', 'once');      % start of a design
    if ~isempty(tok)
        D(end+1).label = strtrim(tok{1}); %#ok<SAGROW>
        D(end).names = {};  D(end).sgn = {};  D(end).lim = [];  D(end).val = zeros(0, 2);
        continue
    end
    if isempty(D), continue, end
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
for i = 1:nD
    bad = {};
    for k = 1:numel(D(i).names)
        v = D(i).val(k, :);  L = D(i).lim(k);
        if strcmp(D(i).sgn{k}, '<='), excess = max(v)/L - 1;     % worst motor
        else,                         excess = 1 - min(v)/L;
        end
        t = 0;
        if startsWith(D(i).names{k}, 'Ms'),      t = tol.Ms;      useMs(i) = max(v)/L; end
        if startsWith(D(i).names{k}, 'chatter'), t = tol.chatter; useCh(i) = max(v)/L; end
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
best = min(cols(acc, :), [], 1);
score = mean(cols./best, 2);
[~, idx] = sort(score(acc));
acc = acc(idx);

fprintf('\n=========== ACCEPTED (Ms <= +%g %%, chatter <= +%g %%), best first ===========\n', ...
        100*tol.Ms, 100*tol.chatter);
fprintf('%-28s %5s %6s %6s | %13s %13s %13s %6s\n', 'design', 'score', 'Ms', 'chat.', ...
        'joint [deg]', 'tip [mm]', 'contour [mm]', 'HF [V]');
fprintf('%-28s %5s %6s %6s | %13s %13s %13s %6s\n', '', '', 'use', 'use', ...
        '3165 / 3166', 'RMS / max', 'RMS / max', '');
for i = acc'
    fprintf('%-28s %5.2f %5.0f%% %5.0f%% | %6.3f/%6.3f %6.3f/%6.3f %6.3f/%6.3f %6.3f\n', ...
        D(i).label, score(i), 100*useMs(i), 100*useCh(i), M(i, 1:7));
end

rej = find(~ok);
if ~isempty(rej)
    fprintf('\n=========== REJECTED ===========\n');
    for i = rej'
        fprintf('%-28s %s\n', D(i).label, why{i});
    end
end

if ~isempty(acc)
    i = acc(1);
    fprintf(['\nBest: %s  (score %.2f; uses %.0f %% of the Ms limit and %.0f %% of the ' ...
             'chatter limit)\n'], D(i).label, score(i), 100*useMs(i), 100*useCh(i));
    fprintf('Score: 1.00 = best of the accepted designs on joint error, tip RMS, contour RMS and max.\n');
end


%% ---------------- local functions ----------------
function m = run_metrics(file, t_skip, f_hf)
% Joint error, tip tracking, contour error and HF voltage of one saved run.
S = load(file);
r1 = get_ts(S, 'r_3165');   y1 = get_ts(S, 'y_3165');   u1 = get_ts(S, 'u_3165');
r2 = get_ts(S, 'r_3166');   y2 = get_ts(S, 'y_3166');   u2 = get_ts(S, 'u_3166');
[xd, t] = get_ts(S, 'xc_des');   yd = get_ts(S, 'yc_des');
xs = get_ts(S, 'xc_sim');        ys = get_ts(S, 'yc_sim');
k  = t >= t_skip;                                   % periodic regime only
Ts = median(diff(t));

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
