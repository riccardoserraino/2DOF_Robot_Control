function ff = ff_correct(model, ff_source, run_file, t_skip)
% FF_CORRECT  Feedforward identified on a run. Works for any controller (PP, PID, ...)
%             whose Simulink model has the standard feedforward blocks.
%
% HOW TO USE (after the design script: it writes gains and the default _ff values)
%   ff_correct('Model_pp_obs', 'sim')          simulate, fit, write, simulate again
%   ff_correct('Model_pid_ff', 'sim')          the same for the PID (after pid_tune.m)
%   ff_correct([], 'run', 'lab/CNC_pp_obs_<date>.mat')   fit on a robot run, write
%   ff_correct([], 'model')                    nothing to fit: keep the default _ff values
%
%   ff_source : 'model', 'sim' or 'run' (see above)
%   run_file  : run to fit for 'run' (name in data/, data/sim, data/lab, or full path)
%   t_skip    : [s] the fit starts after t_skip (default 5.5 s = start + one lap)
%   ff(i)     : .k .tau .Vc .R2 of motor i (1 = 3165, 2 = 3166), also written in the
%               workspace as k_xxxx_ff, tau_xxxx_ff, Vc_xxxx_ff (read by the models)
%
% THEORY (details in docs/ff_fit_explained.pdf)
%   Once the robot follows the path (y ~ phi), the logged voltage u is the voltage the
%   motor needs to follow phi, whatever controller produced it. We write it as
%       u = c1*dphi + c2*ddphi + c3*tanh(dphi/w_eps) + c4
%           speed     accel.    friction              offset
%   and find c1..c4 by least squares. Comparing with the feedforward of the models
%       u_ff = -(1/k)*dphi - (tau/k)*ddphi - Vc*tanh(dphi/w_eps)
%   gives  k = -1/c1,  tau = c2/c1,  Vc = -c3.
%   The feedforward is outside the loop: poles and margins do not change.

if nargin < 3, run_file = ''; end
if nargin < 4, t_skip = 5.5;  end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..'));                    % scripts/: run_and_save
data_dir = fullfile(here, '..', '..', 'data');

% same tanh width and saturation as the model (written by parameters.m)
w_eps = evalin('base', 'w_eps');                  % [rad/s]
Vmax  = evalin('base', 'Vmax');                   % [V]

%% 1. The run to fit
ff = [];
if strcmp(ff_source, 'model')
    disp('ff_source = model: the default feedforward stays, nothing to fit.');
    return
elseif strcmp(ff_source, 'sim')
    disp('First run with the current feedforward ...');
    run_file = [run_and_save(model) '.mat'];      % saved in data/sim
elseif ~strcmp(ff_source, 'run')
    error('ff_source must be ''model'', ''sim'' or ''run''.');
end
file = find_file(run_file, data_dir);
S = load(file);
fprintf('\nFeedforward fit on %s\n', file);

%% 2. Fit, one motor at a time
motors = {'3165', '3166'};
for i = 1:2
    m = motors{i};

    % signals of the run: reference [deg] and controller voltage [V]
    [t, phi] = get_signal(S, ['r_' m]);
    [tu, u]  = get_signal(S, ['u_' m]);
    u = interp1(tu, u, t);                        % same time instants as phi

    % speed and acceleration of the reference, in rad (as in the model)
    dphi  = gradient(phi, t) * pi/180;
    ddphi = gradient(dphi, t);

    % samples to use: after the start, and where the amplifier is not saturated
    use = (t > t_skip) & (abs(u) < 0.99*Vmax);

    % and before the disturbance test, if the run has one (d is not part of the plant)
    if has_signal(S, ['d_' m])
        [td, d] = get_signal(S, ['d_' m]);
        if any(d ~= 0)
            use = use & (t < td(find(d ~= 0, 1)));
        end
    end

    % least squares:  u = X*c
    X = [dphi, ddphi, tanh(dphi/w_eps), ones(size(t))];
    c = X(use, :) \ u(use);

    % from the coefficients to the feedforward parameters
    k   = -1/c(1);
    tau = c(2)/c(1);
    Vc  = -c(3);

    % quality of the fit: R2 = 1 - (unexplained part)/(total), 1 = perfect
    res = u(use) - X(use, :)*c;
    R2  = 1 - sum(res.^2) / sum((u(use) - mean(u(use))).^2);

    % feedforward used in the run, to show what changes
    [k_old, tau_old, Vc_old] = old_feedforward(S, m);
    u_old = -(1/k_old)*dphi - (tau_old/k_old)*ddphi - Vc_old*tanh(dphi/w_eps);
    fb_old = rms(u(use) - u_old(use));            % voltage the feedback had to supply
    fb_new = rms(res);                            % what it will still have to supply

    fprintf('motor %s   (R2 = %.3f)\n', m, R2);
    fprintf('  k   %.4f -> %.4f rad/(V s)\n', k_old, k);
    fprintf('  tau %.4f -> %.4f s\n', tau_old, tau);
    fprintf('  Vc  %.3f -> %.3f V\n', Vc_old, Vc);
    fprintf('  voltage left to the feedback (RMS): %.3f -> %.3f V\n', fb_old, fb_new);

    % write the new feedforward for the models
    assignin('base', ['k_'   m '_ff'], k);
    assignin('base', ['tau_' m '_ff'], tau);
    assignin('base', ['Vc_'  m '_ff'], Vc);
    ff(i).k = k;  ff(i).tau = tau;  ff(i).Vc = Vc;  ff(i).R2 = R2;
end

%% 3. Simulation: run again with the new feedforward
if strcmp(ff_source, 'sim')
    disp('Final run with the fitted feedforward ...');
    tag = run_and_save(model);
    fprintf('saved: data/sim/%s.mat (add it to validate_controllers.m)\n', tag);
end
end

%% ---------------- local functions ----------------
function file = find_file(name, data_dir)
% the run file: full path, or under data/, data/sim, data/lab
places = {name, fullfile(data_dir, name), fullfile(data_dir, 'sim', name), ...
          fullfile(data_dir, 'lab', name)};
for p = 1:numel(places)
    if isfile(places{p}), file = places{p}; return, end
end
error('Run file "%s" not found in data/, data/sim, data/lab.', name);
end

function [t, x] = get_signal(S, name)
% one logged signal as two columns (time, value). run_and_save may add the run
% tag to the name (r_3165_<tag>); the signal is a timeseries or a structure with time.
if ~isfield(S, name)
    name = [name '_' S.run_tag];
end
v = S.(name);
if isa(v, 'timeseries')
    t = v.Time;   x = v.Data;
else
    t = v.time;   x = v.signals.values;
end
t = double(t(:));  x = double(x(:));
[t, keep] = unique(t);                            % the solver can repeat a time instant
x = x(keep);
end

function yes = has_signal(S, name)
% true if the run contains the signal (with or without the run tag)
yes = isfield(S, name) || (isfield(S, 'run_tag') && isfield(S, [name '_' S.run_tag]));
end

function [k, tau, Vc] = old_feedforward(S, m)
% feedforward used in the run: the _ff values saved in it, otherwise the model
k   = S.(['k_' m '_load']);
tau = S.(['tau_' m '_load']);
Vc  = 0;
if isfield(S, ['k_' m '_ff'])
    k   = S.(['k_' m '_ff']);
    tau = S.(['tau_' m '_ff']);
    Vc  = S.(['Vc_' m '_ff']);
end
end
