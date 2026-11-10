%% PARAMETERS - static constants of the CNC plotter (no computations here)
% Loaded by every script with run('../parameters.m') (from a scripts/ subfolder).
% Nothing here is tuned: only datasheet values and identification results.

%% 1. Motor + gearbox (Quanser SRV02 datasheet, high-gear configuration)
Rm = 2.6;              % Armature resistance [Ohm]
Lm = 0.18e-3;          % Armature inductance [H]

kt = 7.68e-3;          % Torque constant [N*m/A]
km = 7.68e-3;          % Back-EMF constant [V/(rad/s)]

Kg = 70;               % Total gear ratio [-]
nm = 0.69;             % Motor efficiency [-]
ng = 0.90;             % Gearbox efficiency [-]

Jm_rotor = 3.9e-7;     % Rotor inertia, motor side [kg*m^2]
Jeq      = 2.087e-3;   % Equivalent inertia, load side [kg*m^2]
beta_eq  = 0.015;      % Equivalent viscous friction, load side [N*m*s/rad]

%% 2. Limits
fmax      = 50;        % Maximum input frequency [Hz]
Imax      = 1;         % Maximum continuous current [A]
omega_max = 628.3;     % Maximum motor speed [rad/s]
Vmax      = 6;         % Voltage saturation used in the Simulink models [V]

%% 3. Four-bar linkage
m = 0.065;             % Link mass [kg]
l = 0.127;             % Link length, all four links are equal [m]

%% 4. Identified models: G(s) = k / (s*(tau*s + 1)),  k in [rad/(V*s)], tau in [s]
% Results of model_id/identification.m (periodic multisine, procest 'P1I').
% "noload" = motor alone, "load" = four-bar linkage attached.
k_3165_noload   = 1.601288977178586;
tau_3165_noload = 0.020622907170894;
k_3166_noload   = 1.528072828084566;
tau_3166_noload = 0.021821915834148;

k_3165_load     = 1.436697278373122;
tau_3165_load   = 0.037421535883333;
k_3166_load     = 1.381159339691515;
tau_3166_load   = 0.038119941238613;

%% 5. Modeling of the disturbances: plant used in simulation
% Controller and feedforward use *_load (the model we know).
% The simulated motor uses *_plant, so model and "real" plant differ as on the bench.
% Realistic values: fit of the multisine data with delay + Coulomb friction.
%   Td    = pure delay at the motor input [s] (ZOH of the 2 ms controller excluded)
%   Vc    = Coulomb friction as equivalent input voltage [V]
%   w_eps = width of the smooth transition around zero speed [rad/s] (tanh)
use_real_plant = false;     % true -> realistic plant, false -> plant equal to the model
w_eps = 0.02;              % [rad/s] about 1 deg/s; raise to 0.05 if the simulation is slow

if use_real_plant
    k_3165_plant = 1.77;   tau_3165_plant = 0.0428;   Td_3165 = 0.0035;   Vc_3165 = 0.12;
    k_3166_plant = 1.74;   tau_3166_plant = 0.0444;   Td_3166 = 0.0035;   Vc_3166 = 0.14;
else
    k_3165_plant = k_3165_load;   tau_3165_plant = tau_3165_load;   Td_3165 = 0;   Vc_3165 = 0;
    k_3166_plant = k_3166_load;   tau_3166_plant = tau_3166_load;   Td_3166 = 0;   Vc_3166 = 0;
end