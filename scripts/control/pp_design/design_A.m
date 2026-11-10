%% DESIGN A - complex pair + one real pole      (theory: pp_design_theory.pdf, sections 4 and 10)
%   pair      : -zeta*wn +- j*wn*sqrt(1 - zeta^2)    wn = speed, zeta = damping
%   real pole : -alpha*zeta*wn                        alpha < 1 -> slower than the pair
%   beta*     : 1/(1 + 2*alpha*zeta^2) puts the zero of the step on the real pole, so the
%               step behaves like the pair alone: wn^2/(s^2 + 2*zeta*wn*s + wn^2)
%
% WHAT IT DOES: starts from the PID poles (or your numbers), searches the closest poles that
% meet the six requirements on BOTH motors, prints the tables, plots the 10 deg step of each
% motor, and writes the Simulink variables of both motors in the workspace.
% To test poles chosen by hand without a search, use check_design.m.
clear; clc; close all
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));     % make utils/ visible

%% 1. Settings (the only lines to change)
start_from_pid = true;          % true: start from the PID poles; false: from the 3 values below
wn    = 55;                     % [rad/s] speed/stiffness: higher = faster, stiffer joint, more voltage and noise
zeta  = 0.9;                    % [-] damping (0 < zeta < 1): higher = less oscillation/overshoot, a bit slower
alpha = 5/(zeta*wn);            % [-] integral speed: real pole = alpha*zeta*wn, higher = friction/load errors removed faster
beta_mode = 'star';             % 'star' = beta* (steps), 'one' = beta 1 (trajectories)
do_search = true;               % false: no search, keep the start values

%% 2. Motors, PID and requirements
[mot, req] = pp_setup();        % mot(1) = motor 3165, mot(2) = motor 3166

% Trajectory design (beta = 1): the step overshoot does not matter on a path, and with
% beta = 1 the zero of the step always gives overshoot. So that requirement is switched off;
% all the others (settling, voltage, phase margin, chatter, slowest pole) stay.
if strcmp(beta_mode, 'one')
    req.R       = 3;             % the type of deg error we expect on a path
    req.OS_max = Inf;
    req.ts_max = Inf;            % with beta = 1 the step settling fights a fast integral pole
    fprintf('beta = 1 (trajectory design): overshoot requirement switched off.\n');
% Step design
else
    req.R = 3;                  % the type of deg error we expect on a step
    req.OS_max = 2;              % beta* cancels the slow pole, so the pair decides the overshoot
    req.ts_max = 0.2;            % with beta = 1 the step settling fights a fast integral pole
    fprintf('beta = * (step design): overshoot requirement switched on.\n');
end

%% 3. Start values from the PID poles
if start_from_pid
    [wn, zeta, alpha] = from_pid(mot(1).p_pid);
    fprintf('start from the PID: wn = %.2f  zeta = %.3f  alpha = %.3f\n', wn, zeta, alpha);
end

%% 4. Search (pp_search): moves wn, zeta, alpha as little as possible until all is met.
% Note: the PID real pole is slow (about 0.36 rad/s). The requirement "slowest pole >= req.lam_min"
% (pp_setup.m, theory section 6) decides how far it must move: with beta* it is hidden in the
% step, but it still decides how fast the integrator removes load and model errors.
if do_search
    x = pp_search(@poles_A, [wn, zeta, alpha], beta_mode, mot, req);
    wn    = x(1);
    zeta  = min(x(2), 0.999);
    alpha = x(3);
end
p = poles_A([wn, zeta, alpha]);
fprintf('\nresult: wn = %.3f  zeta = %.3f  alpha = %.3f\n', wn, zeta, alpha);

%% 5. Results (both motors) and Simulink variables (both motors)
pp_report(mot, req, p, beta_mode, 'Design A');
pp_simulink(mot, req, p, beta_mode);

%% ---------------- approach A (local functions stay at the end of a script) ----------------
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
% PID poles -> wn, zeta, alpha.  The two fastest PID poles give the pair with the same
% sum and product (same s^2 + 2*zeta*wn*s + wn^2); the slowest PID pole is the real pole.
[~, order] = sort(abs(pp));     % order(1) = index of the slowest pole
slow  = pp(order(1));
fast1 = pp(order(2));
fast2 = pp(order(3));
wn    = sqrt(real(fast1*fast2));             % product of the pair = wn^2
zeta  = -real(fast1 + fast2)/(2*wn);         % sum of the pair = -2*zeta*wn
if zeta > 0.99
    zeta = 0.99;                             % real PID poles: make a pair just below 1
end
alpha = abs(slow)/(zeta*wn);                 % real pole = alpha*zeta*wn
end
