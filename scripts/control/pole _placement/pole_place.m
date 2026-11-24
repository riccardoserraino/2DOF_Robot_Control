%% POLE_PLACE - pole placement + Luenberger observer of both joints (no search)
%
% CONTROLLER (state x = [theta; omega] in encoder degrees, integral eI' = r - theta)
%   u = u_ff + Kx*(x_ref - xhat) - Ki*eI          (u_ff, x_ref built in Simulink)
% CLOSED-LOOP POLES (separation principle: controller and observer chosen independently)
%   controller : roots of s^2 + 2*zeta*wn*s + wn^2   (complex pair if zeta < 1, two real
%                poles if zeta > 1)   and   -lambda (integral action)
%   integral   : lambda = ifac*zeta*wn, a multiple of the real part of the pair (as pp_tune)
%   observer   : -obs_factor x fastest controller pole, never faster than obs_max
% The requirements are only CHECKED (table, * = not met): the poles stay as written here.
% Gains, checks and Simulink variables are computed by pp_core.m, the same function used by
% pp_tune.m: the tuned design and the final design are computed by the same code.

clear; clc; close all
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', '..', 'parameters.m'));   % motor models, limits, simulated plant

%% 1. Settings (the only lines to change)
traj_id = 7;                     % path read by the model (traj_select.m), 7 = traj_sequence

% design parameters: copy them from the design chosen by pp_tune_analyze
% (label wn<wn> z<zeta> i<ifac> obs<obs_factor>, e.g. wn20 z1.2 i0.7 obs3)
des.wn     = 24;                 % [rad/s] speed of the pair: higher = less error, more noise
des.zeta   = 1.1;                % [-]     damping of the pair (> 1: two real poles)
des.ifac   = 0.5;                % [-]     integral pole = ifac x zeta*wn (same as pp_tune)
des.obs_factor = [3; 3];         % [-]     observer poles = obs_factor x fastest controller pole

des.lambda = des.ifac*des.zeta*des.wn;   % [rad/s] integral pole used by pp_core:
                                         %         friction errors removed in ~4/lambda s

cfg.Ts      = 0.002;             % [s]     sample time of QUARC
cfg.obs_max = [0.5; 0.6]/cfg.Ts; % [rad/s] cap on the observer poles
cfg.Td      = 0.5*cfg.Ts + 0.0035; % [s]   loop delay: ZOH (Ts/2) + measured input delay
cfg.beta    = 1;                 % [-]     weight of the angle reference (1 = trajectories)
cfg.R       = 3;                 % [deg]   test step for the peak voltage
cfg.q       = 360/4096;          % [deg]   one encoder count
cfg.w_noise = 100;               % [rad/s] chatter = largest controller gain above this frequency

% requirements (checked, not imposed), in the order of the table
lim    = [4.8;    % [V]     peak voltage on the test step (80 % of Vmax: disturbance margin)
          45;     % [deg]   phase margin, with the delay Td
          1.6;    % [-]     Ms = peak of |1/(1+L)|, with the delay Td (GM >= 2.7, PM >= 36 deg)
          0.1;    % [V]     chatter: voltage jump from one encoder count (vibrations)
          10;     % [rad/s] slowest pole
          40];    % [rad/s] crossover wc (|L| = 1), with the delay Td: upper limit.
                  %         Above ~40 rad/s the loop relies on the model beyond the
                  %         mechanical pole 1/tau (~26 rad/s) and amplifies encoder noise

%% 2. Design (controller model = plant of parameters.m) and Simulink variables
[res, ctrl] = pp_core([k_3165_plant, k_3166_plant], [tau_3165_plant, tau_3166_plant], des, cfg);
mots = {'3165', '3166'};

% CNC_pp_obs.slx still contains an unused PID side loop (PID -> model -> scope) that reads
% kp/ki/kd_xxxx: zero gains only let the model compile. Delete that loop and these lines can go.
for i = 1:2
    assignin('base', ['kp_' mots{i}], 0);
    assignin('base', ['ki_' mots{i}], 0);
    assignin('base', ['kd_' mots{i}], 0);
end

%% 3. Crossover frequency wc (6th requirement)
% Loop opened at the motor input, L = -Cy*G, with the delay Td (same loop as PM and Ms in
% pp_core). wc is where |L(j wc)| = 1: roughly the closed-loop bandwidth of the joint.
for i = 1:2
    G = ss(ctrl(i).A, ctrl(i).B, [1, 0], 0);   G.InputDelay = cfg.Td;
    [~, ~, ~, res(6, i)] = margin(-ctrl(i).Cy*G);          % 4th output = gain crossover
end

%% 4. Table (* = not met)
lab    = {'peak voltage [V]', 'phase margin [deg]', 'Ms [-]', 'chatter [V]', ...
          'slowest pole [rad/s]', 'crossover wc [rad/s]'};
is_min = [false; true; false; false; true; false];   % true = the value must be ABOVE the limit
fprintf('pole_place: wn %g rad/s, zeta %g, lambda = %g x zeta*wn = %.2f rad/s, observer %s x\n\n', ...
        des.wn, des.zeta, des.ifac, des.lambda, mat2str(des.obs_factor.'));
fprintf('%-22s %10s %9s %9s\n', 'requirement', 'limit', '3165', '3166');
for j = 1:numel(lim)
    if is_min(j), sgn = '>='; bad = res(j, :) < lim(j);
    else,         sgn = '<='; bad = res(j, :) > lim(j);
    end
    mark = {' ', ' '};   mark(bad) = {'*'};
    fprintf('%-22s %s %7.3g %8.3f%s %8.3f%s\n', lab{j}, sgn, lim(j), ...
            res(j, 1), mark{1}, res(j, 2), mark{2});
end

%% 5. Poles [rad/s]
% Separation principle: the closed loop has 5 poles = 3 controller poles + 2 observer poles.
% Check: the poles of the whole loop (motor + observer + controller, delay left out) are
% computed directly and must be the same 5 numbers.
pp = roots([1, 2*des.zeta*des.wn, des.wn^2]);          % the pair: s^2 + 2 zeta wn s + wn^2
fprintf('\nPOLES [rad/s]\n');
if des.zeta < 1
    fprintf('  controller pair  : %7.2f +- %6.2fj   (|p| = wn = %g, zeta = %g)\n', ...
            real(pp(1)), abs(imag(pp(1))), des.wn, des.zeta);
else                                                      % two real poles, product = wn^2
    fprintf('  controller pair  : %7.2f  %7.2f   (real, zeta = %g, product = wn^2 = %g)\n', ...
            real(pp(1)), real(pp(2)), des.zeta, des.wn^2);
end
fprintf('  integral pole    : %7.2f   (= %g x zeta*wn)\n', -des.lambda, des.ifac);
for i = 1:2
    po = ctrl(i).p_obs;
    fprintf('  observer %s    : %7.2f  %7.2f    (%.1f and %.1f x wn)\n', ...
            mots{i}, po(1), po(2), abs(po(1))/des.wn, abs(po(2))/des.wn);
end
for i = 1:2
    G0   = ss(ctrl(i).A, ctrl(i).B, [1, 0], 0);           % motor model, no delay
    p_cl = pole(feedback(G0, ctrl(i).Cy, +1));             % u = Cy*y (sign inside Cy)
    [~, o] = sort(abs(p_cl));   p_cl = p_cl(o);
    txt = arrayfun(@(z) sprintf('%.1f%+.1fj', real(z), imag(z)), p_cl, 'UniformOutput', false);
    fprintf('  closed loop %s (check): %s\n', mots{i}, strjoin(txt.', '   '));
end

fprintf('\nSimulink variables written. Run the model.\n');
