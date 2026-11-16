function ff = ff_fit(run_file, mot, Vmax, opt)
% FF_FIT  Feedforward identified on a logged run (simulation or bench).
%
% HOW TO USE
%   ff = ff_fit('Model_pp_obs_20261115_134120.mat', mot, 6, opt);
%   run_file : name (searched in data/, data/sim, data/lab), 'sim/...', 'lab/...' or full path
%   mot      : motors of pp_setup (only k, tau and name are used)
%   opt.t_skip  [s]     skip the start: use only the periodic regime of the trajectory
%   opt.w_eps   [rad/s] width of the tanh, the same as in the model (parameters.m)
%   opt.Vc_run  [V]     friction compensation used in the run, [3165 3166]. Used only for
%                       old run files that do not contain Vc_xxxx_ff.
%   ff(i).k, ff(i).tau, ff(i).Vc   new feedforward of motor i,   ff(i).R2 quality of the fit
%
% THEORY (inverse model identified on the run)
%   In the periodic regime y ~ r, so the logged voltage u is the voltage the plant needs to
%   follow r. Fitting u on signals built from r gives the inverse model of the plant:
%       u ~ c1*dphi + c2*ddphi + c3*tanh(dphi/w_eps) + c4*ddphi_other + c0
%           viscous   inertia   Coulomb friction     coupling with     offset
%                                                    the other joint
%   The feedforward of the Simulink models (dphi, ddphi in rad):
%       u_ff = -(1/k)*dphi - (tau/k)*ddphi - Vc*tanh(dphi/w_eps)
%   so the fit gives directly   k = -1/c1,   tau = c2/c1,   Vc = -c3.
%   What the fit adds to the old feedforward is the voltage the FEEDBACK supplied, and the
%   feedback needs an error to produce it: moved into the feedforward, that error goes away.
%   The regressors come from r (known, noise free), not from y: no noise bias in the fit.
%   Only r and u are used: the fit does not depend on the controller (PP, PID, ...).

data_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', '..', 'data');
f = find_run(run_file, data_dir);
S = load(f);
fprintf('\nFeedforward fit on: %s\n', f);

ff = struct('k', {}, 'tau', {}, 'Vc', {}, 'R2', {});
for i = 1:2
    name  = mot(i).name;
    other = mot(3 - i).name;            % the other joint (for the coupling term)

    % logged signals: reference [deg], controller voltage before saturation [V]
    [t, r] = read_signal(S, ['r_' name]);
    u  = read_signal(S, ['u_' name], t);
    r2 = read_signal(S, ['r_' other], t);
    d  = read_signal(S, ['d_' name], t);
    if isempty(t) || isempty(u) || isempty(r2)
        error('ff_fit: r_%s, u_%s or r_%s missing in %s', name, name, other, f);
    end

    % samples to use: periodic regime, before the disturbance, not saturated
    keep = t >= opt.t_skip & abs(u) < 0.99*Vmax;
    if ~isempty(d) && any(d ~= 0)
        keep = keep & t < t(find(d ~= 0, 1));
    end

    % derivatives of the reference in rad (as dphi, ddphi in the model)
    dr   = gradient(r, t)*pi/180;
    ddr  = gradient(dr, t);
    ddr2 = gradient(gradient(r2, t)*pi/180, t);

    % least squares: u ~ Phi*c
    Phi = [dr, ddr, tanh(dr/opt.w_eps), ddr2, ones(size(t))];
    c   = Phi(keep, :) \ u(keep);

    % back to the form of the Simulink feedforward
    ff(i).k   = -1/c(1);
    ff(i).tau = c(2)/c(1);
    ff(i).Vc  = -c(3);
    ff(i).R2  = 1 - var(u(keep) - Phi(keep, :)*c)/var(u(keep));

    % old feedforward: the _ff values saved in the run, otherwise k_load, tau_load, Vc_run
    k_old = mot(i).k;  tau_old = mot(i).tau;  Vc_old = opt.Vc_run(i);
    if isfield(S, ['k_' name '_ff']),   k_old   = S.(['k_' name '_ff']);   end
    if isfield(S, ['tau_' name '_ff']), tau_old = S.(['tau_' name '_ff']); end
    if isfield(S, ['Vc_' name '_ff']),  Vc_old  = S.(['Vc_' name '_ff']);  end

    % voltage the FEEDBACK has to supply, before and after (the error scales with it)
    u_ff_old = -(1/k_old)*dr - (tau_old/k_old)*ddr - Vc_old*tanh(dr/opt.w_eps);
    fb_old   = rms(u(keep) - u_ff_old(keep));
    fb_new   = rms(u(keep) - Phi(keep, :)*c);

    fprintf('motor %s   (fit R^2 = %.3f)\n', name, ff(i).R2);
    fprintf('  k   : %.4f -> %.4f rad/(V s)\n', k_old, ff(i).k);
    fprintf('  tau : %.4f -> %.4f s\n', tau_old, ff(i).tau);
    fprintf('  Vc  : %.3f -> %.3f V\n', Vc_old, ff(i).Vc);
    fprintf('  (not used) coupling %+.2e V/(rad/s^2),  offset %+.3f V\n', c(4), c(5));
    fprintf('  voltage left to the feedback (RMS): %.3f V -> %.3f V\n', fb_old, fb_new);
end
fprintf(['R^2 close to 1: the residual error was model error -> the new feedforward removes it.\n' ...
         'R^2 low: the rest is noise or effects not in the fit -> faster poles.\n']);
end

%% ---------------- local functions ----------------
function f = find_run(name, data_dir)
% full path, or relative to data/ ('sim/...', 'lab/...'), or name only
cand = {name, fullfile(data_dir, name), fullfile(data_dir, 'sim', name), ...
        fullfile(data_dir, 'lab', name)};
ok = cand(cellfun(@isfile, cand));
if isempty(ok), error('Run file "%s" not found in data/, data/sim, data/lab', name); end
f = ok{1};
end

function [t, x] = read_signal(S, name, t_ref)
% Logged signal "name" (or "name_<tag>" written by run_and_save) as a column.
% Accepts timeseries, structure with time, or plain arrays (2 ms samples).
% With t_ref: returns the values interpolated on t_ref (single output).
v = [];
if isfield(S, name)
    v = S.(name);
elseif isfield(S, 'run_tag') && isfield(S, [name '_' S.run_tag])
    v = S.([name '_' S.run_tag]);
end
t = [];  x = [];
if isempty(v), return, end
if isa(v, 'timeseries')
    t = v.Time;  x = squeeze(v.Data);
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;  x = squeeze(v.signals.values);
else
    x = squeeze(v);  t = (0:numel(x)-1)'*2e-3;
end
t = double(t(:));  x = double(x(:));
[t, iu] = unique(t, 'last');                         % variable-step solvers repeat time stamps
x = x(iu);
if nargin > 2
    t = interp1(t, x, t_ref, 'linear', 'extrap');    % single output: values on t_ref
end
end
