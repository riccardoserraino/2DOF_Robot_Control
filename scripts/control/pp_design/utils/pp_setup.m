function [mot, req] = pp_setup()
% PP_SETUP  Motor models, PID and requirements: the starting point of every script.
%
% HOW TO USE
%   [mot, req] = pp_setup();
%   mot(1) is motor 3165, mot(2) is motor 3166. Each one has:
%       mot(1).k, mot(1).tau        identified model  G(s) = k/(s(tau s + 1))
%       mot(1).A, .B, .C, .G        state-space model in encoder degrees
%       mot(1).Ai, .Bi              the same model with the integral state added
%       mot(1).Cpid, .p_pid         the PID of pid_tune.m and its closed-loop poles
%   req holds every requirement and setting. CHANGE THE NUMBERS ONLY HERE.

%% 1. Requirements (theory pdf, section 5)
req.R       = 10;      % [deg]   size of the reference step
req.OS_max  = 2;       % [%]     maximum overshoot
req.ts_max  = 0.2;     % [s]     maximum settling time (2 % band)
req.Vmax    = 6;       % [V]     maximum voltage = saturation of the amplifier
req.PM_min  = 45;      % [deg]   minimum phase margin (computed with the 3 ms delay)
req.chatter = Inf;     % [V]     maximum voltage ripple caused by one encoder count
req.lam_min = 2;       % [rad/s] slowest pole allowed. Independent of ts_max: the slow pole
                       % can stay close to the PID one (about 2.6 rad/s), while the other
                       % poles stay fast. Lower it to allow an even slower pole.
                       % otherwise set it to 4/req.ts_max
% To neglect requirements instead of deleting them from the scripts, do the
% following:
% req.R       = Inf;      
% req.OS_max  = Inf;      
% req.ts_max  = Inf;     
% req.Vmax    = Inf;      
% req.PM_min  = 0;    
% req.chatter = Inf;    
% req.lam_min = 0;

%% 2. Fixed settings
req.t  = linspace(0, 1.5, 1500)';  % [s]   simulation time, 1500 points (column)
req.d  = 0.5;                      % [V]   load step, only plotted by explore_poles
req.Ts = 0.002;                    % [s]   sample time of the real controller
req.Td = 1.5*req.Ts;               % [s]   delay of the digital controller (theory pdf, section 2)
req.q  = 360/4096;                 % [deg] one encoder count
req.w  = 1e4;                      % weight of the penalty in the search (pp_search)


%% 3. Run pid_tune.m and take k, tau and the PID gains
% run() with a full path goes into the folder of the script while it runs, so the
% relative paths inside pid_tune.m still work. It runs inside the local function
% read_pid_tune (bottom of this file), so its variables do not fill your workspace.
this_folder = fileparts(mfilename('fullpath'));            % .../pp_design/utils
pid_file    = fullfile(this_folder, '..', '..', 'pid_tune.m');
v = read_pid_tune(pid_file);

%% 4. Build the two motor models
mot(1) = make_motor('3165', v.k_3165, v.tau_3165, v.pid_3165);
mot(2) = make_motor('3166', v.k_3166, v.tau_3166, v.pid_3166);

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
% the few variables we need into the struct v.
run(pid_file);
v.k_3165   = k_3165_load;
v.tau_3165 = tau_3165_load;
v.pid_3165 = contr_3165;
v.k_3166   = k_3166_load;
v.tau_3166 = tau_3166_load;
v.pid_3166 = contr_3166;
end