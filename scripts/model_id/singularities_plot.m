%% SINGULARITIES OF THE FOUR-BAR LINKAGE (joint space and Cartesian space)
% Requires FK_classic.m (run kinematics.m first).
%
% Loop-closure constraints |C-B| = l, |C-D| = l, differentiated in time:
%   Jx * dC = Jq * dq,   Jx = [(C-B)'; (C-D)'],
%   Jq = diag((C-B)'*dB/dphi, (C-D)'*dD/dtheta)
% Serial left  : Jq(1,1) = 0 -> link BC aligned with AB (left arm stretched/folded)
% Serial right : Jq(2,2) = 0 -> link DC aligned with ED (right arm stretched/folded)
% Parallel     : det(Jx) = 0, B ~= D -> B, C, D aligned (|BD| = 2l)
% Degenerate   : B = D -> C is undetermined (it can rotate around B = D)
%
% Geometry:
% A = (0,0), E = (2*l,0)
% B = l*[cos(phi); sin(phi)]
% D = [2*l-l*sin(theta); l*cos(theta)]

clear; clc; close all;
run('../parameters.m');

%% 1. Joint-space grid
N = 601;
g = linspace(-pi, pi, N);
[PH, TH] = meshgrid(g, g);

%% 2. Mechanism geometry
Bx = l*cos(PH);
By = l*sin(PH);

Dx = 2*l - l*sin(TH);
Dy = l*cos(TH);

dx = Dx - Bx;
dy = Dy - By;

s2 = (dx.^2 + dy.^2)/l^2;

% Valid circle intersection, excluding B = D.
valid = s2 <= 4 + 1e-9 & s2 > 1e-10;

% Forward kinematics: "+" branch.
s = l*sqrt(s2);
h = l*sqrt(max(1 - s2/4, 0));

Cx = (Bx + Dx)/2 - h.*dy./s;
Cy = (By + Dy)/2 + h.*dx./s;

Cx(~valid) = NaN;
Cy(~valid) = NaN;

%% 3. Constraint singularity factors
u1x = Cx - Bx;
u1y = Cy - By;

u2x = Cx - Dx;
u2y = Cy - Dy;

% Diagonal terms of Jq.
S1 = (u1x.*(-l*sin(PH)) + ...
      u1y.*( l*cos(PH)))/l^2;

S2 = (u2x.*(-l*cos(TH)) + ...
      u2y.*(-l*sin(TH)))/l^2;

% det(Jx)/l^2.
P = (u1x.*u2y - u1y.*u2x)/l^2;

S1(~valid) = NaN;
S2(~valid) = NaN;
P(~valid)  = NaN;

%% 4. Extract singularity curves
Q_left  = get_curves(g, S1, 0);
Q_right = get_curves(g, S2, 0);

% Ordinary parallel singularity: |BD| = 2*l.
Q_parallel = get_curves(g, s2, 4);

% Degenerate configuration B = D:
% phi = 0, theta = pi/2.
q_degenerate = [0, pi/2];

%% 5. Map serial singularities into Cartesian space
X_left  = map_to_cartesian(Q_left,  l);
X_right = map_to_cartesian(Q_right, l);

% Ordinary parallel singularity:
% C is the midpoint of B and D.
% This avoids numerical issues in FK at h = 0.
X_parallel = map_parallel_midpoint(Q_parallel, l);

% Degenerate Cartesian locus:
% B = D = (l,0); C can be anywhere on the circle
% of radius l centred at (l,0).
alpha = linspace(0, 2*pi, 500);
X_degenerate = [l + l*cos(alpha(:)), l*sin(alpha(:))];

%% 6. Joint-space plot
figure('Name', 'Singularities - Joint Space');
hold on; grid on;

plot(Q_left(:,1), Q_left(:,2), ...
    'b-', 'LineWidth', 1.5, ...
    'DisplayName', 'Serial: left');

plot(Q_right(:,1), Q_right(:,2), ...
    'g-', 'LineWidth', 1.5, ...
    'DisplayName', 'Serial: right');

plot(Q_parallel(:,1), Q_parallel(:,2), ...
    'r-', 'LineWidth', 1.5, ...
    'DisplayName', 'Parallel: B,C,D aligned');

plot(q_degenerate(1), q_degenerate(2), ...
    'cp', 'MarkerSize', 16, 'MarkerFaceColor', 'c', ...
    'DisplayName', 'Degenerate: B = D');

plot(0, 0, 'ko', 'MarkerFaceColor', 'y', ...
    'DisplayName', 'Home');

xlabel('\phi [rad]');
ylabel('\theta [rad]');
title('Singularities - Joint Space');
axis equal;
xlim([-pi pi]);
ylim([-pi pi]);
legend('Location', 'best');

%% 7. Cartesian-space plot
figure('Name', 'Singularities - Cartesian Space');
hold on; grid on;

plot(X_left(:,1), X_left(:,2), ...
    'b-', 'LineWidth', 1.5, ...
    'DisplayName', 'Serial: left');

plot(X_right(:,1), X_right(:,2), ...
    'g-', 'LineWidth', 1.5, ...
    'DisplayName', 'Serial: right');

plot(X_parallel(:,1), X_parallel(:,2), ...
    'r-', 'LineWidth', 1.5, ...
    'DisplayName', 'Parallel: midpoint of BD');

plot(X_degenerate(:,1), X_degenerate(:,2), ...
    'c--', 'LineWidth', 2, ...
    'DisplayName', 'B = D: possible positions of C');

plot([0 l l 2*l 2*l], [0 0 l l 0], ...
    'k-o', 'LineWidth', 1.5, ...
    'MarkerFaceColor', 'k', ...
    'DisplayName', 'Home mechanism');

% Reference trajectory (square of the Simulink models: centre (l,l), side 9 cm)
sq = 0.09;
plot(l + sq/2*[-1 1 1 -1 -1], l + sq/2*[-1 -1 1 1 -1], ...
    'm-', 'LineWidth', 2, ...
    'DisplayName', 'Square trajectory');

xlabel('x_C [m]');
ylabel('y_C [m]');
title('Singularities - Cartesian Space');
axis equal;
legend('Location', 'best');

%% ---------------- Local functions ----------------

function Q = get_curves(g, F, level)
% Extract contour points in [phi, theta] coordinates.
% NaN rows separate disconnected curves.

    M = contourc(g, g, F, [level level]);
    Q = zeros(0,2);
    k = 1;

    while k < size(M,2)
        n = M(2,k);
        Q = [Q; M(:,k+1:k+n).'; NaN NaN]; %#ok<AGROW>
        k = k + n + 1;
    end
end


function XY = map_to_cartesian(Q, l)
% Map singularity curves to Cartesian space.
% Reject configurations outside the real workspace.

    XY = NaN(size(Q));

    for k = 1:size(Q,1)
        if ~all(isfinite(Q(k,:)))
            continue;
        end

        phi = Q(k,1);
        theta = Q(k,2);

        B = l*[cos(phi); sin(phi)];
        D = [2*l - l*sin(theta); l*cos(theta)];

        % Check circle-intersection feasibility.
        s2 = sum((D-B).^2)/l^2;

        if s2 > 4 + 1e-9 || s2 < 1e-10
            continue;
        end

        C = FK_classic(phi, theta, l);

        % Avoid plotting complex or non-finite coordinates.
        if isreal(C) && all(isfinite(C))
            XY(k,:) = C(:).';
        end
    end
end


function XY = map_parallel_midpoint(Q, l)
% For a parallel singularity, C is the midpoint of B and D.
% No square root is required, avoiding complex coordinates.

    XY = NaN(size(Q));

    for k = 1:size(Q,1)
        if ~all(isfinite(Q(k,:)))
            continue;
        end

        phi = Q(k,1);
        theta = Q(k,2);

        B = l*[cos(phi); sin(phi)];
        D = [2*l-l*sin(theta); l*cos(theta)];

        XY(k,:) = ((B+D)/2).';
    end
end
