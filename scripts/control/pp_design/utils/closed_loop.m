function r = closed_loop(m, req, ctrl, beta)
% CLOSED_LOOP  Simulates the closed loop of one motor and measures it.
%
% HOW TO USE
%   r = closed_loop(mot(1), req, g, beta)     pole placement (g from pp_gains, beta a number)
%   r = closed_loop(mot(1), req, mot(1).Cpid) PID of pid_tune.m (only 3 inputs)
%   MATLAB counts the inputs in "nargin": 4 inputs -> pole placement, 3 inputs -> PID.
%
% WHAT IS SIMULATED (pp_design_theory.pdf, section 2)
%   ut = u + d          the load d is a voltage added at the motor input
%   delay of 3 ms       the motor model gets InputDelay = Td: the voltage arrives 3 ms late
%   no saturation       the model is linear: the requirement "peak voltage <= 6 V" keeps it valid
% OUTPUT r
%   r.y_r, r.u_r   angle and voltage for a step of req.R degrees in the reference
%   r.y_d          angle for a step of req.d volts in the load (only plotted by explore_poles)
%   r.OS, r.ts, r.u_peak, r.IAE, r.PM, r.GM, r.wc, r.L, r.noise, r.Ms   the numbers

%% 1. Controller block (inputs: reference r and measured angle y, output: voltage u)
if nargin == 4
    K = pp_controller(m, ctrl, beta);
else
    K = pid_controller(ctrl, req.N_pid);
end

%% 2. Motor with the delay
P = m.G;
P.InputDelay = req.Td;               % the voltage reaches the motor Td seconds later

%% 3. Connect: r -> controller -> u -> (+ d) -> motor -> y -> back to the controller
% connect() joins blocks by the names of their signals
P.InputName  = 'ut';
P.OutputName = 'y';
K.InputName  = {'r', 'y'};
K.OutputName = 'u';
add_load = sumblk('ut = u + d');                            % summing junction
CL = connect(P, K, add_load, {'r', 'd'}, {'y', 'u'});       % inputs r, d; outputs y, u

%% 4. Step responses: Y(time, output, input), outputs 1 = y, 2 = u; inputs 1 = r, 2 = d
Y = step(CL, req.t);
r.y_r = req.R*Y(:, 1, 1);            % angle   for a reference step of R degrees
r.u_r = req.R*Y(:, 2, 1);            % voltage for the same step
r.y_d = req.d*Y(:, 1, 2);            % angle   for a load step of d volts

%% 5. Numbers of the step
r.OS = max(0, (max(r.y_r) - req.R)/req.R*100);              % overshoot [%]
outside = abs(r.y_r - req.R) > 0.02*req.R;                  % true where outside the 2 % band
last = find(outside, 1, 'last');                            % last sample outside
if isempty(last)
    r.ts = 0;
else
    r.ts = req.t(last);                                     % settling time [s]
end
r.u_peak = max(abs(r.u_r));                                 % peak voltage [V]
r.IAE    = trapz(req.t, abs(r.y_d));                        % area of the load error [deg s]

%% 6. Margins: loop opened at the motor input, L = -Cy*G with the delay
Cy  = K(1, 2);                       % part of the controller from y (input 2) to u (output 1)
r.L = -Cy*m.G;
r.L.InputDelay = req.Td;
[gm, r.PM, ~, r.wc] = margin(r.L);   % gain margin, phase margin [deg], crossover [rad/s]
r.GM = 20*log10(gm);                 % [dB]
% Ms = peak of |S| = |1/(1 + L)| (with the delay): 1/Ms is the smallest distance of the
% Nyquist curve from -1. Ms <= 1.6 guarantees GM >= 8.5 dB and PM >= 36 deg at the same time.
wM   = logspace(-1, log10(pi/req.Ts), 400);                % 0.1 rad/s .. Nyquist frequency
r.Ms = max(abs(1./(1 + squeeze(freqresp(r.L, wM)))));      % [-]

%% 7. Chatter: one encoder count times the largest controller gain above 100 rad/s
w  = logspace(2, log10(pi/req.Ts), 50);                     % 100 rad/s .. Nyquist frequency
Cw = squeeze(freqresp(Cy, w));                              % controller gain at those frequencies
r.noise = max(abs(Cw))*req.q;                               % [V]
end

%% ---------------- local functions ----------------
function K = pp_controller(m, g, beta)
% Pole placement with observer (theory pdf, section 2):
%   observer   xhat' = A*xhat + B*u + L*(y - C*xhat)
%   integral   eI'   = r - y
%   control    u     = beta*k1*r - Kx*xhat - ki*eI
% Putting u inside the observer gives one block with states [xhat; eI]:
%   [xhat'; eI'] = Ac*[xhat; eI] + Bc*[r; y],     u = Cc*[xhat; eI] + Dc*[r; y]
N  = beta*g.k1;                                   % gain of the reference
Ac = [m.A - m.B*g.Kx - g.L*m.C,  -m.B*g.ki;
      0, 0,                       0      ];
Bc = [m.B*N,  g.L;                                % column 1: input r,  column 2: input y
      1,      -1 ];
Cc = [-g.Kx, -g.ki];
Dc = [N, 0];
K  = ss(Ac, Bc, Cc, Dc);
end

function K = pid_controller(Cpid, N)
% PID of the Simulink block (parallel form, derivative on the error, filter N, no output
% limit):  u = C(s)*r - C(s)*y.  The gains are the ones of pid_tune.m, never changed here.
% N = req.N_pid (pp_setup.m), the value of the PID block in CNC_pid_ff.slx.
% Note: the derivative acts on the error, so a reference step gives a short voltage spike
% u(0+) = R*(Kp + Kd*N) lasting about 1/N. In the real loop it is cut at 6 V.
C = Cpid;
C.Tf = 1/N;                                       % derivative filter
C = ss(C);
K = [C, -C];
end
