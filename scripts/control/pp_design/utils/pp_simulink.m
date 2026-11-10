function pp_simulink(mot, req, p, beta)
% PP_SIMULINK  Writes in your workspace the variables read by Model_pp_obs.slx and
%              CNC_pp_obs.slx, for BOTH motors (the models always use both).
%
% HOW TO USE
%   pp_simulink(mot, req, p, 'star')      (mot must contain both motors)
%   beta: 'star', 'one' or a number
%
% Variables created (xxxx = 3165 and 3166, so the two motors never overwrite each other):
%   Kx_xxxx              state feedback [k1 k2]
%   Ki_xxxx              integral gain
%   Kaw_xxxx             anti-windup gain (acts only when the voltage saturates)
%   A_obs_xxxx, B_obs_xxxx   observer  xhat' = A_obs*xhat + B_obs*[u; y]
%   beta_xxxx            reference weight (Gain block on phi before the Mux)
%   lamf_xxxx            pole of the optional reference prefilter lamf/(s + lamf)
%   k_xxxx_load, tau_xxxx_load, kp_xxxx, ki_xxxx, kd_xxxx   model and PID
%   Vmax
% and, before them, every variable of scripts/parameters.m, run in your workspace. Its
% section 5 holds what the simulated plant needs (use_real_plant, w_eps, k_xxxx_plant,
% tau_xxxx_plant, Td_xxxx, Vc_xxxx): Model_pp_obs.slx reads them too.

%% 0. parameters.m in the base workspace (utils -> pp_design -> control -> scripts)
params_file = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'parameters.m');
evalin('base', sprintf('run(''%s'');', params_file));

for i = 1:length(mot)
    m = mot(i);
    g = pp_gains(m, p);
    if ischar(beta)
        b = g.beta.(beta);
    else
        b = beta;
    end

    Kaw   = -mean(abs(p))/g.ki;       % anti-windup with time constant 1/mean|p|
    A_obs = m.A - g.L*m.C;
    B_obs = [m.B, g.L];

    % assignin('base', name, value) creates "name" in your workspace.
    % ['Kx_' m.name] joins two texts: 'Kx_' and '3165' -> 'Kx_3165'
    assignin('base', ['Kx_'    m.name], g.Kx);
    assignin('base', ['Ki_'    m.name], g.ki);
    assignin('base', ['Kaw_'   m.name], Kaw);
    assignin('base', ['A_obs_' m.name], A_obs);
    assignin('base', ['B_obs_' m.name], B_obs);
    assignin('base', ['beta_'  m.name], b);
    % optional reference prefilter F(s) = lamf/(s + lamf) on phi (used with beta = 1):
    % lamf = a0/a1 puts the filter pole on the zero of the step -> step without overshoot
    a = real(poly(p));
    assignin('base', ['lamf_'  m.name], a(4)/a(3));
    assignin('base', ['k_'   m.name '_load'], m.k);
    assignin('base', ['tau_' m.name '_load'], m.tau);
    assignin('base', ['kp_' m.name], m.Cpid.Kp);
    assignin('base', ['ki_' m.name], m.Cpid.Ki);
    assignin('base', ['kd_' m.name], m.Cpid.Kd);
end
assignin('base', 'Vmax', req.Vmax);
fprintf('\nSimulink variables written for both motors (beta = %.3f). Now run the model.\n', b);
end
