function [v, val, lim, names] = pp_check(r, p, req)
% PP_CHECK  The six requirements of the design (theory pdf, section 5).
%
% HOW TO USE
%   r = closed_loop(mot(1), req, g, beta);
%   [v, val, lim, names] = pp_check(r, p, req);
%   val(k)  measured value of requirement k        lim(k)  its limit
%   v(k)    relative violation:  v <= 0  met,   v = 0.1  10 % beyond the limit
%           (all six on the same scale, so the search can add them up)

names = {'overshoot [%]';
         'settling 2% [s]';
         'peak voltage [V]';
         'phase margin [deg]';
         'chatter [V]';
         'slowest pole [rad/s]'};

val = [r.OS;  r.ts;  r.u_peak;  r.PM;  r.noise;  min(abs(p))];
lim = [req.OS_max;  req.ts_max;  req.Vmax;  req.PM_min;  req.chatter;  req.lam_min];

v = zeros(6, 1);
v(1) = val(1)/lim(1) - 1;            % overshoot     must be below its limit
v(2) = val(2)/lim(2) - 1;            % settling      must be below its limit
v(3) = val(3)/lim(3) - 1;            % peak voltage  must be below its limit
v(4) = 1 - val(4)/lim(4);            % phase margin  must be ABOVE its limit
v(5) = val(5)/lim(5) - 1;            % chatter       must be below its limit
v(6) = 1 - val(6)/lim(6);            % slowest pole  must be ABOVE its limit (faster)

for k = 1:6
    if isnan(v(k))                   % a computation that failed counts as violated
        v(k) = 1;
    end
end
end