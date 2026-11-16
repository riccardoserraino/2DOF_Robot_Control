%% DESIGN PP - pole placement (design A) + feedforward correction, in one script
%   theory: docs/pp_design_theory.pdf, docs/pp_design_guide.pdf, ../ff_correct.m (feedforward)
%
% POLES (design A pattern)
%   pair      : -zeta*wn +- j*wn*sqrt(1 - zeta^2)    wn = speed, zeta = damping
%   real pole : -alpha*zeta*wn                        the integral action
%   The search (pp_search) moves wn, zeta, alpha as little as possible until every
%   requirement of utils/pp_setup.m is met on BOTH motors.
%
% FEEDFORWARD (outside the loop: it does not change poles and margins)
%   u_ff = -(1/k_ff)*dphi - (tau_ff/k_ff)*ddphi - Vc_ff*tanh(dphi/w_eps)
%   ff_source decides where k_ff, tau_ff, Vc_ff come from:
%     'model' : the model (k_load, tau_load) and Vc of parameters.m. No run needed.
%     'sim'   : the script simulates the model once with the 'model' feedforward, fits the
%               inverse model on that run (ff_correct), writes the corrected values and
%               simulates again with them (two runs saved in data/sim).
%     'run'   : fit on a run already saved (run_file), no simulation. This is the case of the
%               real robot: a run on the bench with the 'model' feedforward (or any earlier run
%               with the same trajectory), then the fit on its data.
%   Why a run is always needed: the fit learns what the plant needs and the model does not
%   know. Only data of the plant contain it. In simulation the plant IS the model, so the fit
%   can learn only what was added to it (delay, friction); on the bench it learns the rest.
%
% WHAT IT DOES: design, tables and step plots, Simulink variables of both motors in the
% workspace (gains, observer, feedforward) and, with 'sim', the two simulations.
clear; clc; close all
this = fileparts(mfilename('fullpath'));
addpath(fullfile(this, 'utils'));               % pp_setup, pp_search, pp_gains, ...
addpath(fullfile(this, '..'));                  % scripts/control/: ff_correct (any controller)

%% 1. Settings (the only lines to change)
% poles
start_from_pid = false;         % true: start from the PID poles; false: from the 3 values below
wn    = 45;                     % [rad/s] speed of the pair (the search lowers it if needed)
zeta  = 0.9;                    % [-] damping of the pair
alpha = 12/(zeta*wn);           % [-] real pole = alpha*zeta*wn = 12 rad/s (integral action)
beta_mode = 'one';              % 'one' = trajectories (with feedforward), 'star' = steps
do_search = true;               % false: keep the values above as they are

% feedforward
ff_source = 'sim';              % 'model', 'sim' or 'run' (see the header)
model     = 'Model_pp_obs';     % model simulated with 'sim'
run_file  = 'lab/CNC_pp_obs_<date>.mat';                 % for 'run': e.g. 'lab/CNC_pp_obs_<date>.mat'
t_skip    = 5.5;                % [s] t_hold + one lap: fit only on the periodic regime

%% 2. Motors and requirements
[mot, req] = pp_setup();        % mot(1) = motor 3165, mot(2) = motor 3166
if strcmp(beta_mode, 'one')
    % trajectories: overshoot and settling of a step do not matter (beta = 1 always gives
    % overshoot); voltage, margins, Ms, chatter and slowest pole stay
    req.R      = 3;             % [deg] size of the errors expected on a path
    req.OS_max = Inf;
    req.ts_max = Inf;
else
    % steps: beta* cancels the real pole, the pair decides overshoot and settling
    req.R      = 3;
    req.OS_max = 2;
    req.ts_max = 0.2;
end

%% 3. Poles: start values and search
if start_from_pid
    [wn, zeta, alpha] = from_pid(mot(1).p_pid);
    fprintf('start from the PID: wn = %.2f  zeta = %.3f  alpha = %.3f\n', wn, zeta, alpha);
end
if do_search
    x = pp_search(@poles_A, [wn, zeta, alpha], beta_mode, mot, req);
    wn    = x(1);
    zeta  = min(x(2), 0.999);
    alpha = x(3);
end
p = poles_A([wn, zeta, alpha]);
fprintf('\npoles: wn = %.2f rad/s  zeta = %.3f  real pole = %.2f rad/s\n', wn, zeta, alpha*zeta*wn);

%% 4. Results and Simulink variables (gains, observer, feedforward = model)
pp_report(mot, req, p, beta_mode, 'Design PP');
pp_simulink(mot, req, p, beta_mode);            % also k_ff = k_load, tau_ff = tau_load, Vc_ff = Vc

%% 5. Feedforward correction (with 'sim' also the two simulations)
ff = ff_correct(model, ff_source, run_file, t_skip);

%% ---------------- local functions (they stay at the end of a script) ----------------
function p = poles_A(x)
% x = [wn, zeta, alpha]  ->  the three poles (column)
wn    = x(1);
zeta  = min(x(2), 0.999);       % zeta < 1 keeps the pair complex
alpha = x(3);
sigma = zeta*wn;                % real part of the pair
wd    = wn*sqrt(1 - zeta^2);    % imaginary part of the pair
p = [-sigma + 1i*wd;
     -sigma - 1i*wd;
     -alpha*sigma];
end

function [wn, zeta, alpha] = from_pid(pp)
% PID poles -> wn, zeta, alpha. The two fastest PID poles give the pair with the same sum
% and product; the slowest PID pole is the real pole.
[~, order] = sort(abs(pp));     % order(1) = index of the slowest pole
slow  = pp(order(1));
fast1 = pp(order(2));
fast2 = pp(order(3));
wn    = sqrt(real(fast1*fast2));             % product of the pair = wn^2
zeta  = min(-real(fast1 + fast2)/(2*wn), 0.99);
alpha = abs(slow)/(zeta*wn);                 % real pole = alpha*zeta*wn
end
