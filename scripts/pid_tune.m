%% PID TUNE
s = tf('s');
sys_3165_load = (k_3165_load/(s*(tau_3165_load*s + 1)))*(-360/(2*pi));
sys_3166_load = (k_3166_load/(s*(tau_3166_load*s + 1)))*(-360/(2*pi));


% tuning
wc = 20;
pm = 90;
type = 'PD';
opts = pidtuneOptions('PhaseMargin', pm, 'CrossoverFrequency', wc);
[contr_3165, info_3165] = pidtune(sys_3165_load, type, opts)
[contr_3166, info_3166] = pidtune(sys_3166_load, type, opts)

kp_3165 = contr_3165.Kp;
%ki_3165 = contr_3165.Ki;
kd_3165 = contr_3165.Kd;

kp_3166 = contr_3166.Kp;
%ki_3166 = contr_3166.Ki;
kd_3166 = contr_3166.Kd;
