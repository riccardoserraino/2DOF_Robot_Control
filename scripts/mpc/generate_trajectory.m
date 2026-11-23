function traj = generate_trajectory(shape, Ts, opts)
%GENERATE_TRAJECTORY  Cartesian reference for the CNC pen (geometry + time law only).
%   traj = generate_trajectory(shape, Ts)
%   traj = generate_trajectory(shape, Ts, opts)
%
%   shape : 'triangle' | 'square' | 'circle'
%   Ts    : sample time [s] (must be the controller sample time)
%   opts  : struct, every field optional
%       center    [2x1] centre of the shape [m]                 default [0.127; 0.127] (home)
%       size      side (triangle, square) or diameter (circle) [m]   default 0.09
%       T_seg     duration of each side of a polygon [s]          default 4
%       T_circle  duration of one full circle [s]                 default 12
%       T_app     duration of the approach segment [s], 0 = none  default 2
%       start     [2x1] where the approach starts [m]             default = center (home)
%
%   Every segment (approach, sides, circle) uses a quintic time law, so
%   velocity and acceleration are zero at the start and end of each segment.
%   Polygons are drawn counter-clockwise; the circle starts at its lowest point.
%
%   Output struct (n samples, column k <-> time t(k)):
%       t        [1xn]  time [s]
%       p        [2xn]  position [m]
%       pd, pdd  [2xn]  velocity [m/s] and acceleration [m/s^2]
%       tg, nr   [2xn]  unit tangent and left normal of the GEOMETRIC path
%                       (defined also where the velocity is zero, e.g. corners)
%       seg      [1xn]  segment index (0 = approach)
%       shape, Ts, opts

if nargin < 3, opts = struct(); end
def = struct('center', [0.127; 0.127], 'size', 0.09, 'T_seg', 4, ...
             'T_circle', 12, 'T_app', 2, 'start', []);
fn = fieldnames(def);
for i = 1:numel(fn)
    if ~isfield(opts, fn{i}) || isempty(opts.(fn{i}))
        opts.(fn{i}) = def.(fn{i});
    end
end
if isempty(opts.start), opts.start = opts.center; end
c = opts.center(:);  L = opts.size;  st = opts.start(:);

segs = {};
switch lower(shape)
    case {'square', 'triangle'}
        if strcmpi(shape, 'square')
            V = c + L/2*[-1  1  1 -1;
                         -1 -1  1  1];              % CCW from bottom-left
        else
            R   = L/sqrt(3);                        % circumradius of the equilateral triangle
            ang = deg2rad([210 330 90]);            % bottom-left, bottom-right, top
            V   = c + R*[cos(ang); sin(ang)];
        end
        V = [V, V(:,1)];                            % close the polygon
        segs = add_approach(segs, st, V(:,1), opts.T_app, Ts);
        for j = 1:size(V,2)-1
            segs{end+1} = line_segment(V(:,j), V(:,j+1), opts.T_seg, Ts, j); %#ok<AGROW>
        end

    case 'circle'
        r   = L/2;
        th0 = -pi/2;                                % start at the lowest point
        segs = add_approach(segs, st, c + r*[cos(th0); sin(th0)], opts.T_app, Ts);
        segs{end+1} = circle_segment(c, r, th0, opts.T_circle, Ts);

    otherwise
        error('generate_trajectory: unknown shape "%s" (triangle | square | circle).', shape);
end

% Concatenate the segments and append the final point (at rest)
traj.p   = cell2mat(cellfun(@(s) s.p,   segs, 'UniformOutput', false));
traj.pd  = cell2mat(cellfun(@(s) s.pd,  segs, 'UniformOutput', false));
traj.pdd = cell2mat(cellfun(@(s) s.pdd, segs, 'UniformOutput', false));
traj.tg  = cell2mat(cellfun(@(s) s.tg,  segs, 'UniformOutput', false));
traj.seg = cell2mat(cellfun(@(s) s.seg, segs, 'UniformOutput', false));
last = segs{end};
traj.p   = [traj.p,   last.p_end];
traj.pd  = [traj.pd,  zeros(2,1)];
traj.pdd = [traj.pdd, zeros(2,1)];
traj.tg  = [traj.tg,  last.tg_end];
traj.seg = [traj.seg, last.seg(end)];

traj.nr    = [-traj.tg(2,:); traj.tg(1,:)];     % tangent rotated by +90 deg (left normal)
traj.t     = (0:size(traj.p,2)-1)*Ts;
traj.shape = lower(shape);
traj.Ts    = Ts;
traj.opts  = opts;
end

%% ------------------------------------------------------------------------
function segs = add_approach(segs, a, b, T, Ts)
% Straight approach from a to b, skipped if T = 0 or a = b
if T > 0 && norm(b - a) > 1e-9
    segs{end+1} = line_segment(a, b, T, Ts, 0);
end
end

function S = line_segment(a, b, T, Ts, id)
% Straight segment a -> b in T seconds (last sample excluded, it starts the next segment)
tl = 0:Ts:T-Ts/2;
[s, sd, sdd] = quintic(tl/T);
d  = b - a;
S.p   = a + d*s;
S.pd  = d*sd/T;
S.pdd = d*sdd/T^2;
S.tg  = repmat(d/norm(d), 1, numel(tl));
S.seg = id*ones(1, numel(tl));
S.p_end  = b;
S.tg_end = d/norm(d);
end

function S = circle_segment(c, r, th0, T, Ts)
% Full circle, counter-clockwise, angle th0 -> th0 + 2*pi in T seconds
tl = 0:Ts:T-Ts/2;
[s, sd, sdd] = quintic(tl/T);
th   = th0 + 2*pi*s;
thd  = 2*pi*sd/T;
thdd = 2*pi*sdd/T^2;
e_r = [cos(th); sin(th)];            % radial unit vector
e_t = [-sin(th); cos(th)];           % tangent unit vector (CCW)
S.p   = c + r*e_r;
S.pd  = r*thd.*e_t;
S.pdd = r*thdd.*e_t - r*thd.^2.*e_r;
S.tg  = e_t;
S.seg = ones(1, numel(tl));
S.p_end  = c + r*[cos(th0); sin(th0)];
S.tg_end = [-sin(th0); cos(th0)];
end

function [s, sd, sdd] = quintic(tau)
% Quintic time law on tau in [0,1]: s(0)=0, s(1)=1, zero velocity/acceleration at both ends.
% Derivatives are with respect to tau (divide by T and T^2 outside).
s   = 10*tau.^3 - 15*tau.^4 +   6*tau.^5;
sd  = 30*tau.^2 - 60*tau.^3 +  30*tau.^4;
sdd = 60*tau    - 180*tau.^2 + 120*tau.^3;
end
