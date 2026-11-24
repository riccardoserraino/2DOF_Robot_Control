%% DESIGN PP - the final pole placement design of both joints
%
% Builds ONE design, prints its requirement table and writes its variables for
% Model_pp_obs.slx (simulation) and CNC_pp_obs.slx (robot).
% To compare several designs first, use tools/observer_study.m.
%
% Controller: observer + state feedback + integral action + feedforward (model k, tau
% of parameters.m). Its poles are set by 4 numbers, chosen in section 1:
%   wn, zeta   the pair -zeta*wn +- j*wn*sqrt(1 - zeta^2)  (speed and damping of the joint)
%   lambda     the real pole -lambda                       (speed of the integral action)
%   obs_factor observer poles = obs_factor x the fastest pole
% The requirements are only guard rails: if none is violated ('*' in the table),
% the poles are exactly the ones you wrote.

clear; clc; close all
this = fileparts(mfilename('fullpath'));
addpath(fullfile(this, 'utils'));               % pp_setup, pp_search, pp_gains, ...

%% 1. Settings (the only lines to change)
% --- 1a. poles ---
wn     = 35;                    % [rad/s] higher = less error, but more noise and less robust
zeta   = 0.999;                   % [-]     0.8-0.999, higher = less ringing at the corners
lambda = 11;                    % [rad/s] integral action: removes friction errors in ~4/lambda s
alpha  = lambda/(zeta*wn);      % (internal) real pole written as alpha*zeta*wn
beta_mode = 'one';              % 'one' = trajectories, 'star' = steps
start_from_pid = false;         % true: start from the PID poles instead
do_search  = false;              % false: never move the poles
show_plots = false;             % true: step plots of both motors

% --- 1b. observer and sample time ---
Ts = 0.002;                             % [s] sample time of QUARC
my_req.Ts         = Ts;
my_req.obs_factor = [4; 4];             % [-] observer speed / fastest pole (textbook 2-6)
my_req.obs_max    = [0.5; 0.6]/Ts;      % [rad/s] cap: faster is not reliable with Ts = 2 ms
my_req.Td         = 0.5*Ts + 0.0035;    % [s] delay in the loop: ZOH (Ts/2) + input delay

% --- 1c. requirements (guard rails), both beta modes. A line deleted here takes the default
%     of utils/pp_setup.m; Inf (0 for PM_min and lam_min) switches one off ---
my_req.Vmax    = 6;                     % [V]     amplifier saturation on a 3 deg step
my_req.PM_min  = 45;                    % [deg]   phase margin, computed with the delay Td
my_req.chatter = 0.25;                 % [V]     voltage jump caused by one encoder count at high
                                        %         frequency: limits vibrations (see the guide)
my_req.lam_min = 10;                    % [rad/s] slowest pole: integral fast enough for the corners

% --- 1d. requirements that depend on beta_mode ---
if strcmp(beta_mode, 'one')
    % trajectories: overshoot and settling of a step do not matter (beta = 1 always gives
    % overshoot); voltage, margins, Ms, chatter and slowest pole stay
    my_req.OS_max = Inf;                % [%]     overshoot: off
    my_req.ts_max = Inf;                % [s]     settling: off
    my_req.Ms_max = 1.6;                % [-]     peak of |1/(1+L)|: robustness
else
    % steps: beta* cancels the real pole, the pair decides overshoot and settling
    my_req.R      = 10;                  % [deg]   test step
    my_req.OS_max = 2;                  % [%]     overshoot
    my_req.ts_max = 0.2;                % [s]     settling 2 %
    my_req.Ms_max = 1.6;                % [-]     peak of |1/(1+L)|: robustness
end

%% 2. Motors and requirements
[mot, req] = pp_setup();        % mot(1) = 3165, mot(2) = 3166 (plant model), req = defaults
names = fieldnames(my_req);     % the requirements and settings of section 1 ...
for i = 1:numel(names)
    req.(names{i}) = my_req.(names{i});   % ... replace the defaults
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
fprintf('\nmodel (plant of parameters.m): k = %.3f / %.3f, tau = %.4f / %.4f\n', mot(1).k, mot(2).k, mot(1).tau, mot(2).tau);
fprintf('poles: wn = %.2f rad/s  zeta = %.3f  real pole = %.2f rad/s\n', wn, zeta, alpha*zeta*wn);

%% 4. Table and Simulink variables
pp_report(mot, req, p, beta_mode, 'Design PP', show_plots);
pp_simulink(mot, req, p, beta_mode);            % also k/tau_xxxx_load = model (feedforward blocks)


%% local functions
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
