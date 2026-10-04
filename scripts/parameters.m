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

mb = 0.038;            % Additional mass [kg]
lb = 0.1525;           % Length [m]

fmax = 50;              % Maximum frequency [Hz]
Imax = 1;               % Maximum current [A]
omega_max = 628.3;      % Maximum motor speed [rad/s]