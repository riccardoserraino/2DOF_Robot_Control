%% PP_TUNE - compare several pole placement designs in simulation
%
% WHAT IT DOES
%   1. Builds every combination of the lists in section 1 (wn, zeta, integral factor, observer).
%      Integral pole: lambda = ifac * zeta*wn, i.e. a multiple of the real part of the pair
%      (ifac = 1: same real part as the pair; ifac < 1: slower integral, more PM, lower Ms).
%   2. For each one: gains and requirement check with pp_core.m (the same code as
%      pole_place.m), then a simulation of the model saved by run_and_save.
%   3. Writes a report in data/sim/pp_tune_<date>_<time>.txt (read by pp_tune_analyze.m):
%      requirements used, the values of every design on both motors (* = not met), a summary
%      and the "runs" block for validate_controllers.m.
% WHAT TO EXPECT
%   faster observer : smaller error, more phase margin, more chatter (encoder noise on u)
%   higher wn       : smaller error, higher Ms and more chatter (the 4.5 ms delay limits it)
%   zeta >= 1       : two real poles, no ringing from the pair
%   higher ifac     : friction and model errors removed sooner, but less PM / higher Ms
%                     (the integrator adds phase lag near the crossover)
% HOW TO USE
%   1. Choose the path with traj_id below (traj_select.m): the model reads the same
%      variable through its Constant block "traj_id".
%   2. Set the lists of section 1 and run (about 15 s per simulation).
%   3. Run pp_tune_analyze.m right after (it reads the .mat run files), then copy the
%      chosen design into pole_place.m. After the analysis the .mat files can be deleted:
%      the errors are kept in data/sim/analysis_<report>.txt.

clear; clc; close all
this        = fileparts(mfilename('fullpath'));          % .../pp_design (pp_core is here)
scripts_dir = fullfile(this, '..', '..');                % .../scripts
run(fullfile(scripts_dir, 'parameters.m'));              % motor models, limits, simulated plant
slx_dir     = fullfile(scripts_dir, '..', 'slx');
addpath(this, scripts_dir, slx_dir, fullfile(scripts_dir, 'trajectory'));

%% 1. Settings (the only lines to change)
model_name = 'Model_pp_obs';               % simulation model in slx/
traj_id    = 7;                            % path (traj_select.m): 1 square, 2 square_speeds,
                                           % 3 circle, 4 triangle, 5 star, 6 rhombus, 7 traj_sequence
path_time  = true;                         % true : Stop Time = length of the path
                                           % false: keep the Stop Time saved in the model

% designs to compare: every combination of wn, zeta, ifac and the ROWS of obs_list.
% Pre-check of the requirements (4.5 ms delay included): wn >= 30 rarely meets Ms and PM,
% zeta 1-1.2 gives the most margin, low ifac costs the "slowest pole" limit, high ifac costs PM.
wn_list   = [20, 22, 24, 26, 28, 30];  % [rad/s] speed of the joint
zeta_list = [0.9, 1.0, 1.1, 1.2];      % [-]     damping of the pair (> 1: two real poles)
ifac_list = [0.3, 0.4, 0.5];           % [-]     integral pole: lambda = ifac*zeta*wn [rad/s]
obs_list  = [3 3; 4 4; 5 5];           % [-]     one ROW per observer option [pole 1, pole 2],
                                       %         x fastest controller pole

% settings and requirements: the same as pole_place.m
cfg.Ts      = 0.002;                       % [s]     sample time
cfg.obs_max = [0.5; 0.6]/cfg.Ts;           % [rad/s] cap of the two observer poles
cfg.Td      = 0.5*cfg.Ts + 0.0035;         % [s]     loop delay (ZOH + input delay)
cfg.beta    = 1;                           % [-]     angle reference weight
cfg.R       = 3;                           % [deg]   test step for the peak voltage
cfg.q       = 360/4096;                    % [deg]   one encoder count
cfg.w_noise = 100;                         % [rad/s] chatter frequency range starts here
lim = [4.8;   % [V]     peak voltage on the test step: 80 % of Vmax = 6 V
       45;    % [deg]   phase margin, with the delay
       1.6;   % [-]     Ms: GM >= Ms/(Ms-1) = 2.7 and PM >= 2 asin(1/(2 Ms)) = 36 deg
       0.1;   % [V]     chatter: voltage jump from one encoder count (~2 % of Vmax)
       10;    % [rad/s] slowest pole: integral errors gone in ~4/10 = 0.4 s
       40];   % [rad/s] crossover wc: below it the model is reliable and noise is low

%% 2. Designs, model, stop time
lab    = {'peak voltage [V]', 'phase margin [deg]', 'Ms [-]', 'chatter [V]', 'slowest pole [rad/s]', ...
          'crossover wc [rad/s]'};
is_min = [false; true; false; false; true; false];   % true = the value must be ABOVE the limit
k_nom   = [k_3165_plant,   k_3166_plant];            % controller model (nominal plant)
tau_nom = [tau_3165_plant, tau_3166_plant];

if size(obs_list, 2) == 1, obs_list = [obs_list, obs_list]; end
[W, Z, F, O] = ndgrid(wn_list, zeta_list, ifac_list, 1:size(obs_list, 1));
opts = [W(:), Z(:), F(:), obs_list(O(:), :)];         % one row per design: wn, zeta, ifac, f1, f2
nO = size(opts, 1);
load_system(fullfile(slx_dir, [model_name '.slx']));
if path_time
    t_end = traj_select(-1, traj_id);                % length of the chosen path
    set_param(model_name, 'StopTime', num2str(t_end));   % for this session only (not saved)
else
    t_end = str2double(get_param(model_name, 'StopTime'));
    if isnan(t_end)                                  % Stop Time written as an expression
        t_end = evalin('base', get_param(model_name, 'StopTime'));
    end
end

%% 3. Report header
rep = {};
rep{end+1} = sprintf('PP_TUNE   %s', datestr(now)); %#ok<TNOW1,DATST>
rep{end+1} = sprintf('model %s, path traj_id = %d, stop time %.2f s, %d designs = %d simulations', ...
                     model_name, traj_id, t_end, nO, nO);
rep{end+1} = '';
rep{end+1} = 'REQUIREMENTS';
for j = 1:numel(lim)
    if is_min(j), sgn = '>='; else, sgn = '<='; end
    rep{end+1} = sprintf('  %-22s %s %g', lab{j}, sgn, lim(j)); %#ok<SAGROW>
end
rep{end+1} = sprintf('SETTINGS  Ts = %g ms, Td = %g ms, observer cap = %s rad/s, test step %g deg', ...
                     1e3*cfg.Ts, 1e3*cfg.Td, mat2str(cfg.obs_max.'), cfg.R);
rep{end+1} = '  * = requirement not met';
rep{end+1} = '';
fprintf('%s\n', rep{:});

%% 4. Design, check and simulate every option
runs = cell(0, 3);   summary = cell(nO, 1);
for o = 1:nO
    des.wn = opts(o, 1);  des.zeta = opts(o, 2);  ifac = opts(o, 3);
    des.lambda = ifac*des.zeta*des.wn;               % integral pole [rad/s] for pp_core
    des.obs_factor = opts(o, 4:5).';
    of = opts(o, 4:5);
    if of(1) == of(2), obs_txt = sprintf('%g', of(1)); else, obs_txt = sprintf('%g-%g', of); end
    label = sprintf('wn%g z%g i%g obs%s', des.wn, des.zeta, ifac, obs_txt);

    % gains, requirements and Simulink variables (controller on the nominal model)
    [res, ctrl] = pp_core(k_nom, tau_nom, des, cfg);

    % 6th requirement: crossover wc (same loop as PM and Ms, ~ bandwidth of the joint)
    for i = 1:2
        res(6, i) = crossover(ctrl(i), k_nom(i), tau_nom(i), cfg);
    end
    bad = (res > lim & ~is_min) | (res < lim & is_min);           % 6 x 2

    L = {};
    L{end+1} = sprintf('==== %02d  %s   (wn %g rad/s, zeta %g, lambda = %g x zeta*wn = %.1f rad/s, observer %sx = %s rad/s)', ...
                       o, label, des.wn, des.zeta, ifac, des.lambda, obs_txt, mat2str(round(-ctrl(1).p_obs.')));
    L{end+1} = sprintf('    %-22s %-10s %10s %10s', 'requirement', 'limit', 'motor 3165', 'motor 3166');
    for j = 1:numel(lim)
        if is_min(j), sgn = '>='; else, sgn = '<='; end
        L{end+1} = sprintf('    %-22s %s %-7.3g %9.3f%s %9.3f%s', lab{j}, sgn, lim(j), ...
                           res(j, 1), star(bad(j, 1)), res(j, 2), star(bad(j, 2))); %#ok<SAGROW>
    end

    failed = regexprep(lab(any(bad, 2)), ' \[.*\]', '');        % names without units
    if isempty(failed), verdict = 'all met'; else, verdict = ['NOT met: ' strjoin(failed, ', ')]; end

    % simulation of the design (motor = nominal model)
    try
        tag   = run_and_save(model_name);
        runs(end+1, :) = {[tag '.mat'], 'PP', label}; %#ok<SAGROW>
        files = {[tag '.mat']};
    catch err
        warning('Run "%s" failed: %s', label, err.message);
        files = {['FAILED: ' err.message]};
    end
    L{end+1} = sprintf('    result: %s', verdict);
    L{end+1} = sprintf('    run file(s): %s', strjoin(files, ', '));
    L{end+1} = '';
    fprintf('%s\n', L{:});
    rep = [rep, L]; %#ok<AGROW>
    summary{o} = sprintf('%02d  %-26s %s', o, label, verdict);
end

%% 5. Summary, block for validate_controllers, save
S = [{'================ SUMMARY ================'}, summary(:)', {''}, ...
     {'=== paste into scripts/plot/validate_controllers.m (section "Runs to compare") ==='}, {'runs = {'}];
for k = 1:size(runs, 1)
    S{end+1} = sprintf('  ''%s'',  ''%s'',  ''%s''', runs{k, :}); %#ok<SAGROW>
end
S{end+1} = '};';
fprintf('%s\n', S{:});
rep = [rep, S];
data_sim = fullfile(scripts_dir, '..', 'data', 'sim');
if ~isfolder(data_sim), mkdir(data_sim); end
txt = fullfile(data_sim, ['pp_tune_' datestr(now, 'yyyymmdd_HHMMSS') '.txt']); %#ok<TNOW1,DATST>
fid = fopen(txt, 'w');  fprintf(fid, '%s\n', rep{:});  fclose(fid);
fprintf('Saved in %s\n', txt);


%% ---------------- local functions ----------------
function s = star(b)
% '*' when a requirement is not met, ' ' otherwise
if b, s = '*'; else, s = ' '; end
end

function wc = crossover(c, k, tau, cfg)
% Crossover frequency wc [rad/s] (|L| = 1) of the loop opened at the motor input,
% L = -Cy*G with the delay Td: same loop used by pp_core for PM and Ms.
G = ss([0, 1; 0, -1/tau], [0; -k*180/pi/tau], [1, 0], 0);
G.InputDelay = cfg.Td;
[~, ~, ~, wc] = margin(-c.Cy*G);
end
