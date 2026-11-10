function x = pp_search(poles_fun, x_start, beta, mot, req)
% PP_SEARCH  Finds the poles closest to the start that meet every requirement.
%
% HOW TO USE (inside design_A / design_B)
%   x = pp_search(@poles_A, [wn, zeta, alpha], 'star', mot, req);
%   poles_fun : the pole pattern of the design script. The @ passes the function itself,
%               so the search can call it: poles_fun(x) returns the three poles.
%   x_start   : start parameters, all positive (A: [wn zeta alpha], B: [l1 l2 l3])
%   beta      : 'star', 'one' or a number
%   x         : the parameters found
%
% THE COST (theory pdf, section 7)
%   J(x) = distance(x) + w * penalty(x)
%   distance = sum over the 3 poles of ( log|p| - log|p_start| )^2
%              -> how far the poles moved. In logs, 10 -> 20 rad/s costs as much as 20 -> 40.
%   penalty  = sum over the 7 requirements of max(0, v)^2
%              -> zero when a requirement is met, grows with the square of the violation.
%   w = 1e4 is large: any violation costs much more than moving the poles, so the search
%   first meets every requirement, then stays as close as possible to the start.
%
% HOW fminsearch WORKS
%   It does not know which pole "should" move. It tries points x near the current one
%   (a triangle/tetrahedron of points, Nelder-Mead method), keeps the one with the lowest J
%   and moves towards it. A pole moves only if that lowers J: if a requirement is violated,
%   the parameters that fix it most cheaply (in log distance) are the ones that change.
%   The algorithm is the same for A and B; only the parameters x differ:
%     A: [wn, zeta, alpha] -> the pair moves as a pair (speed wn, angle zeta), the real pole by alpha
%     B: [l1, l2, l3]      -> each real pole moves on its own

%% 1. Start poles (distance is measured from them)
p_start = poles_fun(x_start);

%% 2. Search in logarithms: y = log(x), x = exp(y) is always positive, so poles stay stable
y = log(x_start);

% fminsearch needs a function of y only: this one-line ("anonymous") function calls
% search_cost with all the other inputs fixed.
cost_of_y = @(y) search_cost(y, poles_fun, p_start, beta, mot, req);

options = optimset('MaxFunEvals', 400, 'Display', 'off');
for attempt = 1:4                                 % restart from the last point if needed
    y = fminsearch(cost_of_y, y, options);
    [~, v] = search_cost(y, poles_fun, p_start, beta, mot, req);
    if max(v) <= 1e-3                             % every requirement met (within 0.1 %)
        break
    end
end
x = exp(y);

if max(v) > 1e-3
    fprintf('\npp_search: some requirements are still not met (see the table).\n');
end
end

%% ---------------- local functions ----------------
function [J, v] = search_cost(y, poles_fun, p_start, beta, mot, req)
p = poles_fun(exp(y));                            % poles for these parameters

% requirements on BOTH motors: keep the worst value of each one
v = -inf(7, 1);
for i = 1:length(mot)
    g = pp_gains(mot(i), p);
    r = closed_loop(mot(i), req, g, beta_number(g, beta));
    v_motor = pp_check(r, p, req);
    v = max(v, v_motor);
end

% distance: compare the poles sorted by speed (slow with slow, fast with fast)
speeds_now   = sort(abs(p));
speeds_start = sort(abs(p_start));
distance = sum((log(speeds_now) - log(speeds_start)).^2);

% penalty: only the violated requirements (v > 0) count
penalty = sum(max(v, 0).^2);

J = distance + req.w*penalty;
end

function b = beta_number(g, beta)
% beta can be given by name ('star', 'one') or as a number
if ischar(beta)
    b = g.beta.(beta);       % e.g. g.beta.star
else
    b = beta;
end
end