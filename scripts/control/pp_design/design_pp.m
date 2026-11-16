%% DESIGN PP - pole placement of both joints (design A)
%
% Three closed-loop poles per joint:
%   pair      : -zeta*wn +- j*wn*sqrt(1 - zeta^2)    wn = speed, zeta = damping
%   real pole : -alpha*zeta*wn                        the integral action
% The search (pp_search) moves wn, zeta, alpha as little as possible until every
% requirement is met on BOTH motors. The requirements are set in section 1 below; the
% ones not written there keep the default value of utils/pp_setup.m.
% Output: table and step plots, and the Simulink variables of both motors in the
% workspace (gains, observer, feedforward from the model). Then run the model.

clear; clc; close all
this = fileparts(mfilename('fullpath'));
addpath(fullfile(this, 'utils'));               % pp_setup, pp_search, pp_gains, ...

%% 1. Settings (the only lines to change)
% poles
start_from_pid = false;         % true: start from the PID poles; false: from the 3 values below
wn    = 60;                     % [rad/s] speed of the pair (the search lowers it if needed)
zeta  = 0.9;                    % [-] damping of the pair
alpha = 15/(zeta*wn);           % [-] real pole = alpha*zeta*wn = 15 rad/s (integral action)
beta_mode = 'one';              % 'one' = trajectories (with feedforward), 'star' = steps
do_search = true;               % false: keep the values above as they are
show_plots = false;             % true: step plots of both motors (the table is always printed)

% requirements common to both beta modes (they replace the defaults of utils/pp_setup.m;
% delete a line to use the default, Inf (or 0 for PM_min, lam_min) to switch one off)
my_req.Vmax    = 6;                     % [V]     saturation of the amplifier
my_req.PM_min  = 45;                    % [deg]   phase margin (with the delay Td)
my_req.chatter = 0.5;                   % [V]     encoder noise on u
my_req.lam_min = 10;                    % [rad/s] slowest pole = speed of the integral action
my_req.Td      = 0.5*0.002 + 0.0035;    % [s]     delay: ZOH (Ts/2) + measured input delay

% requirements that depend on beta_mode
if strcmp(beta_mode, 'one')
    % trajectories: overshoot and settling of a step do not matter (beta = 1 always gives
    % overshoot); voltage, margins, Ms, chatter and slowest pole stay
    my_req.R      = 3;                  % [deg]   size of the errors expected on a path
    my_req.OS_max = Inf;                % [%]     overshoot: off
    my_req.ts_max = Inf;                % [s]     settling: off
    my_req.Ms_max = 1.6;               % [-]     peak of |1/(1+L)|: robustness
else
    % steps: beta* cancels the real pole, the pair decides overshoot and settling
    my_req.R      = 3;                  % [deg]   test step
    my_req.OS_max = 2;                  % [%]     overshoot
    my_req.ts_max = 0.2;                % [s]     settling 2 %
    my_req.Ms_max = 1.6;                % [-]     peak of |1/(1+L)|: robustness
end

%% 2. Motors and requirements
[mot, req] = pp_setup();        % mot(1) = motor 3165, mot(2) = motor 3166, req = defaults
names = fieldnames(my_req);     % the requirements written in section 1 ...
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
fprintf('\npoles: wn = %.2f rad/s  zeta = %.3f  real pole = %.2f rad/s\n', wn, zeta, alpha*zeta*wn);

%% 4. Results and Simulink variables (gains, observer, feedforward = model)
pp_report(mot, req, p, beta_mode, 'Design PP', show_plots);
pp_simulink(mot, req, p, beta_mode);            % also k_ff = k_load, tau_ff = tau_load, Vc_ff = Vc


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
