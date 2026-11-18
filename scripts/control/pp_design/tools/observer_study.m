%% OBSERVER STUDY - compare several pole placement designs in simulation
%
% WHAT IT DOES
%   1. Builds every combination of the lists in section 1 (wn, zeta, real pole, observer).
%   2. For each one: computes the gains (as design_pp.m, without the search), checks the
%      requirements on both motors and simulates Model_pp_obs.slx (saved by run_and_save).
%   3. Writes a report in data/sim/observer_study_<date>_<time>.txt:
%        - the requirements used (copied from section 1)
%        - for every design: the value of each requirement on both motors, * = not met
%        - a summary and the "runs" block to paste into validate_controllers.m
%
% WHAT TO EXPECT (theory in docs/pp_design_guide.pdf)
%   - faster observer: smaller error and more phase margin, but more voltage noise (chatter)
%   - higher wn: smaller error, but higher Ms and more chatter (the 4.5 ms delay limits it)
%   - zeta closer to 1: less ringing at the corners, slightly lower Ms
%   - faster real pole (lambda): friction and model errors removed sooner
%
% HOW TO USE
%   1. In Model_pp_obs.slx choose the trajectory (manual switches) and save the model.
%   2. Set the lists of section 1 and run this script (about 15 s per simulation).
%   3. Paste the "runs" block of the report into validate_controllers.m and run it.
%   No other file is changed.

clear; clc; close all
this = fileparts(mfilename('fullpath'));                  % .../pp_design/tools
pp_dir      = fullfile(this, '..');                       % .../pp_design
scripts_dir = fullfile(this, '..', '..', '..');           % .../scripts
slx_dir     = fullfile(scripts_dir, '..', 'slx');         % .../slx
addpath(fullfile(pp_dir, 'utils'));                       % pp_setup, pp_gains, ...
addpath(scripts_dir);                                     % run_and_save
addpath(slx_dir);                                         % so load_system finds the model

%% 1. Settings (the only lines to change)
model_name = 'Model_pp_obs';           % simulation model in slx/
beta_mode  = 'one';                    % trajectories with feedforward

% designs to compare: every combination of wn, zeta, lambda and the ROWS of obs_list
wn_list     = [35, 37, 39, 41];                % [rad/s] speed of the joint
zeta_list   = [0.7 0.8 0.9, 0.999];            % [-]     damping of the pair
lambda_list = [8, 10, 11];                % [rad/s] real pole (integral action)
obs_list    = [5.5 6.5];                 % [-]     observer: one ROW per option, [factor pole 1, factor pole 2]
                                       %         x fastest controller pole. Equal = double pole,
                                       %         e.g. [5 5; 6 6; 7 7; 5.5 6.5; 6 7.5]

% simulated motor: [k error, tau error] per row (0.2 = +20 %); [0 0] = motor equal to the model.
% Robustness test: add the 4 corners [-0.2 -0.15; -0.2 0.15; 0.2 -0.15; 0.2 0.15].
plant_err = [0, 0];
rob.Ms_max = 2;                        % limits on the WRONG plants (usual minimum robustness)
rob.PM_min = 30;

% requirements and settings: the same as design_pp.m section 1
Ts = 0.002;
my_req.Ts       = Ts;                  % [s]     sample time
my_req.obs_max  = [0.5; 0.6]/Ts;       % [rad/s] cap of the two observer poles
my_req.Td       = 0.5*Ts + 0.0035;     % [s]     delay in the loop (ZOH + input delay)
my_req.Vmax     = 6;                   % [V]     peak voltage on the test step
my_req.PM_min   = 45;                  % [deg]   phase margin
my_req.chatter  = 0.275;               % [V]     voltage jump from one encoder count
my_req.lam_min  = 10;                  % [rad/s] slowest pole
my_req.R        = 3;                   % [deg]   test step
my_req.OS_max   = Inf;                 % [%]     overshoot (off with beta = 1)
my_req.ts_max   = Inf;                 % [s]     settling time (off with beta = 1)
my_req.Ms_max   = 1.6;                 % [-]     peak of |1/(1+L)|

%% 2. List of designs and model
if size(obs_list, 2) == 1, obs_list = [obs_list, obs_list]; end   % one factor -> same for both poles
[W, Z, LA, O] = ndgrid(wn_list, zeta_list, lambda_list, 1:size(obs_list, 1));
opts = [W(:), Z(:), LA(:), obs_list(O(:), :)];   % one row per design: wn, zeta, lambda, f1, f2
nO = size(opts, 1);
nP = size(plant_err, 1);
load_system(fullfile(slx_dir, [model_name '.slx']));

%% 3. Report header: the requirements used
rep = {};
rep{end+1} = sprintf('OBSERVER STUDY   %s', datestr(now)); %#ok<TNOW1,DATST>
rep{end+1} = sprintf('model %s (trajectory as selected in the model), beta = %s', model_name, beta_mode);
rep{end+1} = sprintf('%d designs x %d plant(s) = %d simulations', nO, nP, nO*nP);
rep{end+1} = '';
rep{end+1} = 'REQUIREMENTS (from section 1)';
rep{end+1} = sprintf('  peak voltage      <= %g V   (test step of %g deg)', my_req.Vmax, my_req.R);
rep{end+1} = sprintf('  phase margin      >= %g deg', my_req.PM_min);
rep{end+1} = sprintf('  chatter           <= %g V', my_req.chatter);
rep{end+1} = sprintf('  slowest pole      >= %g rad/s', my_req.lam_min);
rep{end+1} = sprintf('  Ms                <= %g', my_req.Ms_max);
rep{end+1} = sprintf('  overshoot         <= %g %%,  settling <= %g s  (Inf = off)', my_req.OS_max, my_req.ts_max);
rep{end+1} = 'SETTINGS';
rep{end+1} = sprintf('  Ts = %g ms,  delay Td = %g ms,  observer cap = %s rad/s', ...
                     1e3*my_req.Ts, 1e3*my_req.Td, mat2str(my_req.obs_max.'));
if any(plant_err(:))
    rep{end+1} = sprintf('  plant errors [k tau] = %s,  limits on them: Ms <= %g, PM >= %g deg', ...
                         mat2str(plant_err), rob.Ms_max, rob.PM_min);
end
rep{end+1} = '  * = requirement not met';
rep{end+1} = '';
fprintf('%s\n', rep{:});

%% 4. Design, check and simulate every option
runs    = cell(0, 3);                  % file, type, label (validate_controllers format)
summary = cell(nO, 1);
lower_limit = [false; false; false; true; false; true; false];   % PM and slowest pole are minimums

for o = 1:nO
    wn = opts(o, 1);  zeta = opts(o, 2);  lambda = opts(o, 3);  of = opts(o, 4:5);
    if of(1) == of(2), obs_txt = sprintf('%g', of(1)); else, obs_txt = sprintf('%g-%g', of); end
    label = sprintf('wn%g z%g l%g obs%s', wn, zeta, lambda, obs_txt);

    % --- design, as in design_pp.m (no search) ---
    [mot, req] = pp_setup();
    names = fieldnames(my_req);
    for i = 1:numel(names)
        req.(names{i}) = my_req.(names{i});
    end
    req.obs_factor = of(:);                    % factors of the two observer poles
    p = poles_A(wn, zeta, lambda);

    % --- requirements of both motors ---
    val = zeros(7, 2);  bad = false(7, 2);
    for i = 1:2
        g = pp_gains(mot(i), p, req);
        r = closed_loop(mot(i), req, g, g.beta.one);
        [v, val(:, i), lim, rname] = pp_check(r, p, req);
        bad(:, i) = v > 1e-3;
    end
    L = {};
    L{end+1} = sprintf('==== %02d  %s   (wn %g rad/s, zeta %g, real pole %g rad/s, observer %sx = %s rad/s)', ...
                       o, label, wn, zeta, lambda, obs_txt, mat2str(round(-g.p_obs.')));
    L{end+1} = sprintf('    %-22s %-10s %10s %10s', 'requirement', 'limit', 'motor 3165', 'motor 3166');
    for k = 1:7
        if isinf(lim(k)), continue; end                  % switched off: not printed
        if lower_limit(k), sgn = '>='; else, sgn = '<='; end
        L{end+1} = sprintf('    %-22s %s %-7.3g %9.3f%s %9.3f%s', rname{k}, sgn, lim(k), ...
                           val(k, 1), star(bad(k, 1)), val(k, 2), star(bad(k, 2))); %#ok<SAGROW>
    end

    % --- Ms and phase margin on the wrong plants (only if requested) ---
    rob_bad = false;
    if any(plant_err(:))
        for j = 1:nP
            for i = 1:2
                g = pp_gains(mot(i), p, req);
                [Ms, PM] = wrong_plant(mot(i), g, req, plant_err(j, 1), plant_err(j, 2));
                b1 = Ms > rob.Ms_max + 1e-3;  b2 = PM < rob.PM_min - 1e-3;
                rob_bad = rob_bad || b1 || b2;
                L{end+1} = sprintf('    plant k %+3.0f%% tau %+3.0f%%, motor %s:  Ms %.2f%s  PM %.1f%s', ...
                    100*plant_err(j, 1), 100*plant_err(j, 2), mot(i).name, Ms, star(b1), PM, star(b2)); %#ok<SAGROW>
            end
        end
    end

    % --- verdict of this design ---
    failed = rname(any(bad, 2));
    if isempty(failed) && ~rob_bad
        verdict = 'all met';
    else
        failed = regexprep(failed, ' \[.*\]', '');       % names without units
        if rob_bad, failed{end+1} = 'robustness'; end %#ok<AGROW>
        verdict = ['NOT met: ' strjoin(failed(:)', ', ')];
    end

    % --- Simulink variables and one run per plant ---
    pp_simulink(mot, req, p, beta_mode);
    files = {};
    for j = 1:nP
        for i = 1:2       % only the simulated motor changes, the controller keeps its model
            assignin('base', ['k_'   mot(i).name '_plant'], mot(i).k  *(1 + plant_err(j, 1)));
            assignin('base', ['tau_' mot(i).name '_plant'], mot(i).tau*(1 + plant_err(j, 2)));
        end
        name = label;
        if any(plant_err(:))
            name = sprintf('%s k%+.0f tau%+.0f', label, 100*plant_err(j, 1), 100*plant_err(j, 2));
        end
        try
            tag = run_and_save(model_name);
            runs(end+1, :) = {[tag '.mat'], 'PP', name}; %#ok<SAGROW>
            files{end+1} = [tag '.mat']; %#ok<SAGROW>
        catch err
            warning('Run "%s" failed: %s', name, err.message);
            files{end+1} = ['FAILED: ' err.message]; %#ok<SAGROW>
        end
    end
    L{end+1} = sprintf('    result: %s', verdict);
    L{end+1} = sprintf('    run file(s): %s', strjoin(files, ', '));
    L{end+1} = '';
    fprintf('%s\n', L{:});
    rep = [rep, L]; %#ok<AGROW>
    summary{o} = sprintf('%02d  %-26s %s', o, label, verdict);
end

%% 5. Summary and the block for validate_controllers
S = {'================ SUMMARY ================'};
S = [S, summary(:)'];
S{end+1} = '';
S{end+1} = '=== paste into scripts/plot/validate_controllers.m (section "Runs to compare") ===';
S{end+1} = 'runs = {';
for k = 1:size(runs, 1)
    S{end+1} = sprintf('  ''%s'',  ''%s'',  ''%s''', runs{k, :}); %#ok<SAGROW>
end
S{end+1} = '};';
fprintf('%s\n', S{:});
rep = [rep, S];

data_sim = fullfile(scripts_dir, '..', 'data', 'sim');
if ~isfolder(data_sim), mkdir(data_sim); end
txt = fullfile(data_sim, ['observer_study_' datestr(now, 'yyyymmdd_HHMMSS') '.txt']); %#ok<TNOW1,DATST>
fid = fopen(txt, 'w');
fprintf(fid, '%s\n', rep{:});
fclose(fid);
fprintf('Saved in %s\n', txt);


%% ---------------- local functions ----------------
function s = star(b)
% '*' when a requirement is not met, ' ' otherwise
if b, s = '*'; else, s = ' '; end
end

function p = poles_A(wn, zeta, lambda)
% Pattern A: pair -zeta*wn +- j*wn*sqrt(1 - zeta^2) and real pole -lambda (column)
zeta = min(zeta, 0.999);               % keep the pair complex
sigma = zeta*wn;
wd    = wn*sqrt(1 - zeta^2);
p = [-sigma + 1i*wd;
     -sigma - 1i*wd;
     -lambda];
end

function [Ms, PM] = wrong_plant(m, g, req, dk, dtau)
% Ms and phase margin when the controller (built on the nominal model m) drives a motor
% with k*(1 + dk) and tau*(1 + dtau). Loop opened at the motor input: L = -Cy*Gp with Td.
% Controller from y to u:  [xhat'; eI'] = Ac*[xhat; eI] + By*y,   u = Cc*[xhat; eI]
Ac = [m.A - m.B*g.Kx - g.L*m.C,  -m.B*g.ki;
      0, 0,                       0      ];
By = [g.L; -1];
Cc = [-g.Kx, -g.ki];
Cy = ss(Ac, By, Cc, 0);
Gp = tf(-m.k*(1 + dk)*180/pi, [m.tau*(1 + dtau), 1, 0]);   % same units as m.G (encoder deg)
Lp = -Cy*Gp;
Lp.InputDelay = req.Td;
[~, PM] = margin(Lp);
w  = logspace(-1, log10(pi/req.Ts), 400);
Ms = max(abs(1./(1 + squeeze(freqresp(Lp, w)))));
end
