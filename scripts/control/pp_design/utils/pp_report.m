function pp_report(mot, req, p, beta, name)
% PP_REPORT  Requirement table (terminal) and step plots for the poles p.
%
% HOW TO USE
%   pp_report(mot, req, p, 'star', 'Design A')      both motors
%   pp_report(mot(2), req, p, 0.7, 'Check')         only motor 3166
%   beta: 'star', 'one' or a number (the design choice, marked in the title)
%
% For every motor given it prints one table and draws one figure with three cases:
%   PID,  beta = beta*,  beta = 1   (beta changes only the step, not the poles,
%   theory pdf section 4). '*' in the table marks a requirement not met.

labels = {'PID', 'beta*', 'beta=1'};

for i = 1:length(mot)
    m = mot(i);
    g = pp_gains(m, p);
    b = beta_number(g, beta);

    % the three simulations (cell array: R{1} ... R{3})
    R = cell(1, 3);
    R{1} = closed_loop(m, req, m.Cpid);
    R{2} = closed_loop(m, req, g, g.beta_star);
    R{3} = closed_loop(m, req, g, 1);
    P = {m.p_pid, p, p};             % poles of each case

    % check every case
    V = zeros(6, 3);
    X = zeros(6, 3);
    for k = 1:3
        [V(:, k), X(:, k), lim, names] = pp_check(R{k}, P{k}, req);
    end

    % print the table
    fprintf('\n%s - motor %s\n', name, m.name);
    fprintf('poles = %s,  beta* = %.3f,  beta used = %.3f\n', mat2str(p.', 4), g.beta_star, b);
    fprintf('%-22s %9s %9s %9s %9s\n', 'requirement', labels{1}, labels{2}, labels{3}, 'limit');
    for j = 1:6
        fprintf('%-22s', names{j});
        for k = 1:3
            if V(j, k) <= 0
                mark = ' ';
            else
                mark = '*';
            end
            fprintf(' %8.2f%s', X(j, k), mark);
        end
        fprintf(' %9.2f\n', lim(j));
    end

    plot_steps(m, req, R, labels, p, name, b);
end
fprintf('\n* = not met. Phase margin and slowest pole are minimum values, the others maximum.\n');
end

%% ---------------- local functions ----------------
function plot_steps(m, req, R, labels, p, name, b)
grey = [0.6 0.6 0.6];   blue = [0 0.45 0.74];   orange = [0.85 0.33 0.10];
colors = {grey, blue, orange};
styles = {'-', '-', '--'};

figure('Name', [name ' - motor ' m.name], 'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.2 0.85 0.5]);
tiledlayout(1, 3, 'TileSpacing', 'compact');

% 1) poles
nexttile;  hold on;  grid on;  box on
plot(real(m.p_pid), imag(m.p_pid), 'x', 'Color', grey, 'MarkerSize', 11, 'LineWidth', 2);
plot(real(p), imag(p), 'o', 'Color', blue, 'MarkerFaceColor', blue, 'MarkerSize', 9);
xline(-req.lam_min, 'k:');
xlabel('Re [rad/s]');  ylabel('Im [rad/s]');  title('poles');
legend('PID', 'pole placement', 'slowest allowed', 'Location', 'best');

% 2) angle
nexttile;  hold on;  grid on;  box on
for k = 1:3
    plot(req.t, R{k}.y_r, styles{k}, 'Color', colors{k}, 'LineWidth', 1.5);
end
yline(req.R*0.98, 'k:');   yline(req.R*1.02, 'k:');           % 2 % band
xlabel('t [s]');  ylabel('\theta [deg]');  xlim([0 1]);
title(sprintf('%s, motor %s: %g deg step (design beta = %.2f)', name, m.name, req.R, b));
legend(labels, 'Location', 'southeast');

% 3) voltage
nexttile;  hold on;  grid on;  box on
for k = 1:3
    plot(req.t, R{k}.u_r, styles{k}, 'Color', colors{k}, 'LineWidth', 1.5);
end
yline(req.Vmax, 'r--');   yline(-req.Vmax, 'r--');            % saturation
xlabel('t [s]');  ylabel('u [V]');  title('voltage');  xlim([0 0.5]);
end

function b = beta_number(g, beta)
% beta can be given by name ('star', 'one') or as a number
if ischar(beta)
    b = g.beta.(beta);
else
    b = beta;
end
end