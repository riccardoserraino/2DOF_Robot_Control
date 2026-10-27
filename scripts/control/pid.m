%% PID DESIGN + WORKSPACE LOADER
% Run this script (from scripts/control) before simulating CNC_pid_ff.slx or
% Model_pid_ff.slx: it loads every parameter the models need and computes
% the PID gains of both motors.

%% 0. Load the workspace
clear; clc; close all;
run('../parameters.m');     % motor constants + identified k, tau (both motors)

%% 1. Plant: identified model with the link attached, output in encoder degrees
% G(s) = k/(s*(tau*s + 1)) is in rad/V; -180/pi converts to degrees and
% applies the encoder sign convention (V > 0 gives a negative angle).
s = tf('s');
sys_3165_load = k_3165_load/(s*(tau_3165_load*s + 1)) * (-180/pi);
sys_3166_load = k_3166_load/(s*(tau_3166_load*s + 1)) * (-180/pi);

%% 2. Tuning specifications (same for both motors)
% wc = 20 rad/s : below the mechanical pole 1/tau ~ 26 rad/s and well inside
%                 the identified band, keeps the voltage away from saturation.
% pm = 90 deg   : no overshoot, so neither axis overshoots the path corners.
% Same specs on both motors -> same closed-loop dynamics on both axes, so
% the coupled five-bar does not distort the shape in the xy-plane.
wc       = 20;              % Crossover frequency [rad/s]
pm       = 90;              % Phase margin [deg]
pid_type = 'PID';           % (not 'type': that would shadow a built-in)
opts = pidtuneOptions('PhaseMargin', pm, 'CrossoverFrequency', wc, ...
                      'DesignFocus', 'reference-tracking');

[contr_3165, info_3165] = pidtune(sys_3165_load, pid_type, opts);
[contr_3166, info_3166] = pidtune(sys_3166_load, pid_type, opts);

%% 3. Gains used by the PID blocks in Simulink
kp_3165 = contr_3165.Kp;  ki_3165 = contr_3165.Ki;  kd_3165 = contr_3165.Kd;
kp_3166 = contr_3166.Kp;  ki_3166 = contr_3166.Ki;  kd_3166 = contr_3166.Kd;

fprintf('3165: Kp = %.4f  Ki = %.4f  Kd = %.4f  (PM = %.1f deg, wc = %.1f rad/s)\n', ...
        kp_3165, ki_3165, kd_3165, info_3165.PhaseMargin, info_3165.CrossoverFrequency);
fprintf('3166: Kp = %.4f  Ki = %.4f  Kd = %.4f  (PM = %.1f deg, wc = %.1f rad/s)\n', ...
        kp_3166, ki_3166, kd_3166, info_3166.PhaseMargin, info_3166.CrossoverFrequency);
