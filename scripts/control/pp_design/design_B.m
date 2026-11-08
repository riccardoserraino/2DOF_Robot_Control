%% DESIGN B - three real poles      (theory: pp_design_theory.pdf, section 7)
%   poles  : -l1, -l2, -l3            all real: no oscillating mode
%   beta*  : the zero of the step cancels the slowest pole (steps)
%   beta 1 : no weight on the reference (trajectories with feedforward)
%
% WHAT IT DOES: starts from the PID poles (or your speeds), searches the closest three real
% poles that meet the six requirements on BOTH motors, prints the tables, plots the 10 deg step
% of each motor, and writes the Simulink variables of both motors in the workspace.
% To test poles chosen by hand without a search, use check_design.m.
clear; clc; close all
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));     % make utils/ visible

%% 1. Settings (the only lines to change)
start_from_pid = true;          % true: start from the PID poles; false: from lambda below
lambda = [10, 20, 35];          % [rad/s] the three speeds
beta_mode = 'one';             % 'star' = beta* (steps), 'one' = beta 1 (trajectories)
do_search = true;               % false: no search, keep the start values

%% 2. Motors, PID and requirements
[mot, req] = pp_setup();        % mot(1) = motor 3165, mot(2) = motor 3166

%% 3. Start values from the PID poles (a complex PID pair becomes two real poles of the same speed)
if start_from_pid
    lambda = abs(mot(1).p_pid);
    lambda = sort(lambda)';     % ' makes it a row
    fprintf('start from the PID: lambda = %s rad/s\n', mat2str(lambda, 4));
end

%% 4. Search (pp_search): moves each speed as little as possible until all is met.
% Each real pole moves on its own; the slowest one is pushed above 8 rad/s by the
% requirement "slowest pole" (theory section 5).
if do_search
    lambda = pp_search(@poles_B, lambda, beta_mode, mot, req);
end
p = poles_B(lambda);
fprintf('\nresult: lambda = %s rad/s\n', mat2str(sort(lambda), 4));

%% 5. Results (both motors) and Simulink variables (both motors)
pp_report(mot, req, p, beta_mode, 'Design B');
pp_simulink(mot, req, p, beta_mode);

%% ---------------- approach B (local function stays at the end of the script) ----------------
function p = poles_B(lambda)
% three speeds -> three real poles (column, from slow to fast)
lambda = sort(lambda);
p = [-lambda(1);
     -lambda(2);
     -lambda(3)];
end
