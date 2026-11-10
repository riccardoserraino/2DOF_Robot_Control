%% CHECK DESIGN - test poles chosen by hand (no search)
% Give the poles in one of two ways:
%   mode = 'params' : wn, zeta, alpha  ->  pair -zeta*wn +- j*wn*sqrt(1-zeta^2), real pole -alpha*zeta*wn
%                     (the same three poles for both motors)
%   mode = 'poles'  : the three poles directly, one set PER MOTOR
%                     (one complex pair + one real pole, or three real poles)
% Choose beta and the motor(s) to check. The script prints the requirement table, plots the
% 10 deg step (PID, beta*, beta = 1) and writes the Simulink variables of BOTH motors in the
% workspace (the models need both).
% Theory: pp_design_theory.pdf.
clear; clc; close all
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));     % make utils/ visible

%% 1. Settings (the only lines to change)
mode = 'params';                % 'params' or 'poles'

% mode 'params' (same poles for both motors)
wn    = 26.886;                 % [rad/s] speed/stiffness: higher = faster, stiffer joint, more voltage and noise
zeta  = 0.992;                  % [-] damping (0 < zeta < 1): higher = less oscillation/overshoot, a bit slower
alpha = 0.011;                  % [-] integral speed: real pole = alpha*zeta*wn, higher = friction/load errors removed faster

% mode 'poles' (one set of three poles per motor)
p_3165 = [-33.72+10.44j -33.72-10.44j -0.36];          % motor 3165, e.g. [-31.2+10.8i -31.2-10.8i -7]
p_3166 = [-32.84+9.64j -32.84-9.64j -0.36];          % motor 3166 (can be different from 3165)

beta = 'one';                  % 'star' = beta* (steps), 'one' = 1 (trajectories), or a number
motors_to_check = [1 2];        % table and plot for: 1 = motor 3165, 2 = motor 3166, [1 2] = both

%% 2. Motors, PID and requirements
[mot, req] = pp_setup();        % mot(1) = motor 3165, mot(2) = motor 3166

%% 3. The three poles of each motor (P{1} for 3165, P{2} for 3166)
P = cell(1, 2);
if strcmp(mode, 'params')
    sigma = zeta*wn;                        % real part of the pair
    wd    = wn*sqrt(1 - zeta^2);            % imaginary part of the pair
    p = [-sigma + 1i*wd;
         -sigma - 1i*wd;
         -alpha*sigma];
    P{1} = p;
    P{2} = p;
elseif strcmp(mode, 'poles')
    P{1} = p_3165(:);                       % (:) makes it a column
    P{2} = p_3166(:);
else
    error('check_design: mode must be ''params'' or ''poles''.');
end

% theory check: poles must be stable (negative real part) and complex poles in pairs
for i = 1:2
    if any(real(P{i}) >= 0)
        error('check_design: motor %s, every pole needs a negative real part.', mot(i).name);
    end
    if abs(sum(imag(P{i}))) > 1e-9
        error('check_design: motor %s, a complex pole needs its conjugate.', mot(i).name);
    end
end

%% 4. Requirements and step plots of the chosen motor(s), each with its own poles
for i = motors_to_check
    pp_report(mot(i), req, P{i}, beta, 'Check');
end

%% 5. Simulink variables of both motors (names end in _3165 and _3166, no overwriting)
for i = 1:2
    pp_simulink(mot(i), req, P{i}, beta);
end