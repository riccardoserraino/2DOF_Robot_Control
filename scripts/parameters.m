%% ELECTRICAL MOTOR PARAMETERS

Rm = 2.6;              % Motor resistance [Ohm]
Lm = 0.18e-3;          % Motor inductance [H]

kt = 7.68e-3;          % Torque constant [N*m/A]
km = 7.68e-3;          % Back-EMF constant [V/(rad/s)]

Kg = 70;               % Gear ratio
nm = 0.69;             % Motor efficiency
ng = 0.90;             % Gearbox efficiency

Jm_rotor = 3.9e-7;     % Motor rotor inertia [kg*m^2]
Jeq = 2.087e-3;        % Equivalent load inertia [kg*m^2]

beta_eq = 0.015;       % Equivalent viscous friction [N*m*s/rad]

m = 0.065;            % Link mass [kg]
l = 0.127;            % Link length [m]

fmax = 50;              % Maximum frequency [Hz]
Imax = 1;               % Maximum current [A]
omega_max = 628.3;      % Maximum motor speed [rad/s]


%% Estimated data
k_3165_noload   = 1.601288977178586;
tau_3165_noload = 0.020622907170894;
k_3166_noload   = 1.528072828084566;
tau_3166_noload = 0.021821915834148;

k_3165_load   = 1.436697278373122;
tau_3165_load = 0.037421535883333;
k_3166_load   = 1.381159339691515;
tau_3166_load = 0.038119941238613;
