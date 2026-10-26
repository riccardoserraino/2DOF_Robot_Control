function [u, xhat] = ss_controller(phi_ref_deg, dphi_ref, u_ff, meas_deg, P)
%SS_CONTROLLER  Kalman observer + state feedback with integral action (one motor).
% Code for a MATLAB Function block in CNC.slx, executed every Ts = 2 ms.
% Paste it in the block (one block per motor); P is a block PARAMETER (see guide).
%
% Inputs
%   phi_ref_deg  joint reference [deg]  (Input3165 / Input3166, already in deg in the model)
%   dphi_ref     joint speed reference [rad/s]  (Goto 'dphi' / 'dtheta' from the IK)
%   u_ff         feedforward voltage [V]  (sum of the two feedforward gains, without PID)
%   meas_deg     encoder angle [deg]  (Measure3165 / Measure3166)
%   P            struct from ss_simulink_init.m: Ad, Bd, Kx, Ki, Ts, u_max
% Outputs
%   u            voltage to the motor [V], already saturated
%   xhat         [theta_hat; omega_hat; d_hat] for logging (Scope / To Workspace)

persistent xh eI                           % observer state and integral, kept between steps
if isempty(xh)
    xh = [meas_deg*pi/180; 0; 0];          % start from the measured angle, at rest
    eI = 0;
end

% Units: the state-space design works in rad (encoder sign kept)
theta   = meas_deg*pi/180;
phi_ref = phi_ref_deg*pi/180;
x_ref   = [phi_ref; dphi_ref];

% 1. Control law with the current estimate x_hat(k|k-1)
%    u = u_ff - Kx (x_hat - x_ref) - Ki eI - d_hat
%    (feedforward + LQR/pole placement + integral action + disturbance compensation)
v = u_ff - P.Kx*(xh(1:2) - x_ref) - P.Ki*eI - xh(3);

% 2. Saturation and anti-windup (integrate only when not saturated)
u = min(max(v, -P.u_max), P.u_max);
if u == v
    eI = eI + P.Ts*(phi_ref - theta);      % eI = integral of (phi_ref - theta), forward Euler
end

% 3. Kalman observer update for the next step, with the SATURATED u
%    x_hat(k+1|k) = Ad x_hat(k|k-1) + Bd [u; theta]
xhat = xh;
xh   = P.Ad*xh + P.Bd*[u; theta];
end
