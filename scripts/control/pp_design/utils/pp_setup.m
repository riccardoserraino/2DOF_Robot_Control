function [mot, req] = pp_setup()
% PP_SETUP  Motor models, PID and requirements: the starting point of every script.
%
% HOW TO USE
%   [mot, req] = pp_setup();
%   mot(1) is motor 3165, mot(2) is motor 3166. Each one has:
%       mot(1).k, mot(1).tau        model G(s) = k/(s(tau s + 1)) = k/tau_xxxx_plant of parameters.m
%       mot(1).A, .B, .C, .G        state-space model in encoder degrees
%       mot(1).Ai, .Bi              the same model with the integral state added
%       mot(1).Cpid, .p_pid         the PID of pid_tune.m and its closed-loop poles
%                                   (only READ, never retuned: it is the reference to compare with)
%   req holds every requirement (section 1) and setting (section 2). These are only the
%   DEFAULTS: design_pp.m section 1 replaces every field it writes. Set values there.

%% 1. Requirements (theory pdf, section 6)
% DEFAULTS: design_pp.m section 1 replaces the ones it writes. To switch one off put
% Inf (0 for PM_min and lam_min).
req.R       = 3;       % [deg]   test step (overshoot and settling are relative to it)
req.OS_max  = 2;       % [%]     maximum overshoot
req.ts_max  = 0.2;     % [s]     maximum settling time (2 % band)
req.Vmax    = 6;       % [V]     maximum voltage = saturation of the amplifier
req.PM_min  = 45;      % [deg]   minimum phase margin (computed with the delay Td)
req.chatter = 0.5;     % [V]     encoder noise on u: one count times the controller HF gain
req.Ms_max  = 1.6;     % [-]     peak of |1/(1+L)| with the delay: robustness to all model errors
                       %         (1.4 good, 1.6 acceptable, 2 minimum). Ms <= 1.6 -> GM >= 8.5 dB, PM >= 36 deg
req.lam_min = 10;      % [rad/s] slowest closed-loop pole (= speed of the integral action):
                       %         how fast friction, loads and model errors are removed

%% 2. Settings (not requirements: they define the controller and the simulation)
% observer: pole i = obs_factor(i) * fastest controller pole, at most obs_max(i)
req.Ts         = 0.002;            % [s]     sample time of the real controller (QUARC)
req.obs_factor = [6; 6];           % [-]     2-6 is the textbook range (see pp_gains.m)
req.obs_max    = [0.5; 0.6]/req.Ts;% [rad/s] 250 and 300: what a 2 ms Euler (ode1) observer follows
% delay and simulation
req.Td = 0.5*req.Ts + 0.0035;      % [s]   ZOH (Ts/2) + measured input delay (3.5 ms, parameters.m)
req.t  = linspace(0, 1.5, 1500)';  % [s]   simulation time, 1500 points (column)
req.d  = 0.5;                      % [V]   load step, only plotted by explore_poles
req.q  = 360/4096;                 % [deg] one encoder count
req.w  = 1e4;                      % weight of the penalty in the search (pp_search)
req.N_pid = 100;                   % [rad/s] derivative filter of the PID block in CNC_pid_ff.slx
                                   %         (used only to simulate the PID for the comparison)

%% 3. Run pid_tune.m and take the PID gains (only for the comparison)
% run() with a full path goes into the folder of the script while it runs, so the
% relative paths inside pid_tune.m still work. It runs inside the local function
% read_pid_tune (bottom of this file), so its variables do not fill your workspace.
this_folder = fileparts(mfilename('fullpath'));            % .../pp_design/utils
pid_file    = fullfile(this_folder, '..', '..', 'pid_tune.m');
v = read_pid_tune(pid_file);

%% 4. Build the two motor models from the plant of parameters.m
% The controller (gains, observer, feedforward) uses k_xxxx_plant, tau_xxxx_plant:
% what they are (realistic plant or the load model) is chosen in parameters.m section 5.
P = read_parameters(fullfile(this_folder, '..', '..', '..', 'parameters.m'));
mot(1) = make_motor('3165', P.k_3165, P.tau_3165, v.pid_3165);
mot(2) = make_motor('3166', P.k_3166, P.tau_3166, v.pid_3166);

% two guide lines drawn by explore_poles (they are not requirements)
req.p_max  = 3/max(mot(1).tau, mot(2).tau);
req.wn_max = 1.5/max(mot(1).tau, mot(2).tau);
end

%% ---------------- local functions (only this file can call them) ----------------
function m = make_motor(name, k, tau, Cpid)
% Model of one motor (theory pdf, section 2):
%   tau*theta'' + theta' = b*u,   b = -k*180/pi
%   k is in rad/(V s); 180/pi converts to degrees; the minus sign is the encoder direction.
b = -k*180/pi;

m.name = name;
m.k    = k;
m.tau  = tau;
m.b    = b;

% state x = [theta; omega]:   theta' = omega,   omega' = -omega/tau + (b/tau)*u
m.A = [0, 1;
       0, -1/tau];
m.B = [0;
       b/tau];
m.C = [1, 0];                         % only the angle is measured
m.G = ss(m.A, m.B, m.C, 0);           % the same model as a MATLAB system object

% integral state added: z = [theta; omega; eI],  eI' = r - theta  (r = 0 for the design)
m.Ai = [m.A,  [0; 0];
        -m.C,  0   ];
m.Bi = [m.B;
        0  ];

% PID of pid_tune.m. With r = 0 it is the state feedback u = -[Kp, Kd, -Ki]*z,
% so its closed-loop poles are the eigenvalues of Ai - Bi*[Kp, Kd, -Ki].
m.Cpid  = Cpid;
K_pid   = [Cpid.Kp, Cpid.Kd, -Cpid.Ki];
m.p_pid = eig(m.Ai - m.Bi*K_pid);
end

function v = read_pid_tune(pid_file)
% Runs pid_tune.m here (its "clear" only clears this small function) and copies
% the two PID controllers into the struct v (only used for the comparison).
run(pid_file);
v.pid_3165 = contr_3165;
v.pid_3166 = contr_3166;
end

function P = read_parameters(params_file)
% Runs parameters.m here (its variables stay in this small function) and copies the
% plant model, the one the controller is designed on, into the struct P.
run(params_file);
P.k_3165 = k_3165_plant;   P.tau_3165 = tau_3165_plant;
P.k_3166 = k_3166_plant;   P.tau_3166 = tau_3166_plant;
end
