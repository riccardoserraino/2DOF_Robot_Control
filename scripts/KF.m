%% KF (tempo CONTINUO) per un motore SRV02 + scansione di Q con la log-verosimiglianza
% Prima di lanciare, in workspace devono esserci:
%   K, tau        parametri della G(s) = K / (s (tau s + 1))   (modello scelto)
%   t, u, theta   vettori colonna della stessa lunghezza, registrati sul motore
%                 u = tensione EFFETTIVA (dopo saturazione) [V], theta [rad]
%
% Uscite (in workspace e salvate nel file nome_uscita):
%   theta_hat, omega_hat, d_hat   stati stimati con la Q migliore
%   kf                            modello continuo, guadagno L, Qn, Rn, osservatore (per l'LQG)
%
% Il filtro e' a tempo continuo (lqe): Qn e Rn sono DENSITA' SPETTRALI.
% I dati registrati sono pero' campioni presi ogni Ts: la log-verosimiglianza
% si valuta su quei campioni, usando l'equivalente esatto a campioni del modello
% continuo (formula di Van Loan). Q scelta e filtro finale restano continui.

%% ===== 0. Parametri =====
nome_uscita = 'stime_kf_motore1.mat';   % cambialo per ogni motore/modello

t = t(:); u = u(:); theta = theta(:);
assert(numel(t) == numel(u) && numel(t) == numel(theta), 't, u e theta devono avere la stessa lunghezza');
assert(~any(isnan([t; u; theta])), 'Ci sono NaN nei dati');
Ts = t(2) - t(1);
assert(max(abs(diff(t) - Ts)) < 1e-3*Ts, 'Campionamento non uniforme');

cpr = 4096;                          % conteggi/giro dell'encoder (quadratura)
R   = (2*pi/cpr)^2/12;               % varianza di quantizzazione per campione [rad^2]
Rn  = R*Ts;                          % densita' spettrale equivalente per lqe [rad^2 s]

%% ===== 1. Modello del motore dalla G(s) =====
G = minreal(tf(K, [tau 1 0]));
[A, B, C, D] = ssdata(ss(G));
assert(abs(D) < 1e-9, 'D non nullo');
Mo = obsv(A, C);                     % cambio di coordinate: stato = [theta; omega]
A = Mo*A/Mo;  B = Mo*B;  C = C/Mo;
n = size(A,1);

%% ===== 2. Stato esteso con disturbo (random walk) =====
% x = [theta; omega; d],  d entra come tensione aggiuntiva (u + d)
Aa = [A, B; zeros(1,n), 0];
Ba = [B; 0];
Ca = [C, 0];
assert(rank(obsv(Aa, Ca)) == n+1, 'Sistema aumentato non osservabile');
x0 = [theta(1); 0; 0];

% Equivalente a campioni del modello continuo: serve SOLO per la verosimiglianza
sysd = c2d(ss(Aa, Ba, Ca, 0), Ts, 'zoh');

%% ===== 3. Omega di riferimento offline e campioni "in moto" =====
% Derivata non causale (solo per scegliere i campioni utili)
win       = max(5, round(0.05/Ts));                 % finestra di ~50 ms
omega_ref = gradient(smoothdata(theta, 'sgolay', win), Ts);

soglia = 0.5;                        % [rad/s] sotto, la quantizzazione falsa il test
n_skip = round(0.5/Ts);              % scarta il transitorio iniziale del filtro
mask   = abs(omega_ref) > soglia;
mask(1:min(n_skip, end)) = false;
fprintf('Campioni usati per la verosimiglianza: %d (%.0f%%)\n', ...
        nnz(mask), 100*nnz(mask)/numel(mask));

%% ===== 4. Scansione di Q_omega e Q_d (densita' spettrali) =====
% Valori di partenza tipici: Q_w ~ 2.5e-2 [(rad/s)^2/s], Q_d ~ 0.9 [V^2/s].
% Se il massimo cade sul bordo della griglia, allargala in quella direzione.
qw_grid = logspace(-6, 2, 25);       % [(rad/s)^2 / s]
qd_grid = logspace(-5, 3, 25);       % [V^2 / s]

logL = nan(numel(qw_grid), numel(qd_grid));
for i = 1:numel(qw_grid)
    for j = 1:numel(qd_grid)
        Qn = diag([0, qw_grid(i), qd_grid(j)]);
        try
            logL(i,j) = log_verosimiglianza(sysd, Aa, Qn, R, Ts, u, theta, t, x0, mask);
        catch
            % Riccati non risolvibile per questa Q: resta NaN
        end
    end
end

[Lmax, idx] = max(logL(:));
[ib, jb]    = ind2sub(size(logL), idx);
fprintf('Massimo sulla griglia:  Q_omega = %.3g, Q_d = %.3g\n', qw_grid(ib), qd_grid(jb));

% Raffinamento: la griglia ha passo di 1/3 di decade, quindi cerco il massimo
% esatto partendo dal punto migliore della griglia (in log10, per stare su scala giusta)
negL  = @(p) meno_logL(p, sysd, Aa, R, Ts, u, theta, t, x0, mask);
p_opt = fminsearch(negL, log10([qw_grid(ib), qd_grid(jb)]));
Qw_opt = 10^p_opt(1);
Qd_opt = 10^p_opt(2);
Lmax   = -negL(p_opt);
Qbest  = diag([0, Qw_opt, Qd_opt]);

fprintf('Q migliore (continua):  Qn = diag([0 %.4g %.4g]),  Rn = %.4g,  logL = %.8g\n', ...
        Qw_opt, Qd_opt, Rn, Lmax);

% --- Plot della log-verosimiglianza ---
Lrel = logL - Lmax;                  % 0 nel punto migliore, negativo altrove
figure('Name', 'Log-verosimiglianza');
subplot(2,2,[1 3]);
% Le differenze vanno da 0 a milioni: colori in scala logaritmica della distanza
% dal massimo, altrimenti la zona attorno all'ottimo appare tutta dello stesso colore.
Lshow = -log10(1 - Lrel);            % 0 = ottimo, -1 = 10 sotto, -2 = 100 sotto, ...
contourf(log10(qd_grid), log10(qw_grid), Lshow, 40, 'LineStyle', 'none');
colormap(gca, sky);                  % chiaro = lontano, scuro = vicino all'ottimo
                                     % (sky esiste da R2023a; prima usa flipud(bone))
cb = colorbar;
tk = 0:-1:floor(min(Lshow(:)));
cb.Ticks = fliplr(tk);
cb.TickLabels = arrayfun(@(k) sprintf('-10^{%d}', -k), fliplr(tk), 'UniformOutput', false);
cb.TickLabels{end} = '0';
cb.Label.String = 'logL - logL_{max} (scala log)';
hold on; plot(log10(Qd_opt), log10(Qw_opt), 'rp', ...
              'MarkerSize', 14, 'MarkerFaceColor', 'r');
xlabel('log_{10} Q_d'); ylabel('log_{10} Q_\omega');
title('Log-verosimiglianza delle innovazioni');

subplot(2,2,2);
semilogx(qd_grid, Lrel(ib,:), 'o-'); grid on;
xline(Qd_opt, 'r--');
xlabel('Q_d  [V^2/s]'); ylabel('logL - logL_{max}');
title('Sezione a Q_\omega ottimo');

subplot(2,2,4);
semilogx(qw_grid, Lrel(:,jb), 'o-'); grid on;
xline(Qw_opt, 'r--');
xlabel('Q_\omega  [(rad/s)^2/s]'); ylabel('logL - logL_{max}');
title('Sezione a Q_d ottimo');

%% ===== 5. Filtro CONTINUO con la Q migliore =====
L = lqe(Aa, eye(n+1), Ca, Qbest, Rn);
disp('Poli dell''osservatore (Aa - L*Ca):'); disp(eig(Aa - L*Ca).');

% Osservatore come sistema LTI continuo. Ingressi: [u; y]  Uscite: [theta_hat; omega_hat; d_hat]
obs  = ss(Aa - L*Ca, [Ba, L], eye(n+1), 0);
xhat = lsim(obs, [u, theta], t - t(1), x0, 'zoh');

figure('Name', 'Stime');
subplot(3,1,1); plot(t, theta, t, xhat(:,1)); legend('misura','\theta stimato'); ylabel('\theta');
subplot(3,1,2); plot(t, xhat(:,2));           ylabel('\omega stimato');
subplot(3,1,3); plot(t, xhat(:,3));           ylabel('d stimato [V]'); xlabel('t [s]');

%% ===== 6. Output: stati stimati e filtro, pronti per l'LQG =====
theta_hat = xhat(:,1);               % [rad]
omega_hat = xhat(:,2);               % [rad/s]
d_hat     = xhat(:,3);               % [V] disturbo equivalente in tensione

kf.Aa = Aa;  kf.Ba = Ba;  kf.Ca = Ca;  % modello continuo, stato [theta; omega; d]
kf.L  = L;                           % dx_hat/dt = Aa x_hat + Ba u + L (y - Ca x_hat)
kf.Qn = Qbest;  kf.Rn = Rn;          % densita' spettrali usate
kf.obs = obs;                        % osservatore continuo, ingressi [u; y]
kf.Qw_opt = Qw_opt;  kf.Qd_opt = Qd_opt;  kf.logL = Lmax;
kf.K = K;  kf.tau = tau;             % modello del motore da cui e' partito il filtro

save(nome_uscita, 't', 'u', 'theta', 'theta_hat', 'omega_hat', 'd_hat', 'kf');
fprintf('Stime salvate in %s (theta_hat, omega_hat, d_hat, kf)\n', nome_uscita);

%% ===== Funzioni locali =====
function logL = log_verosimiglianza(sysd, Aa, Qn, R, Ts, u, y, t, x0, mask)
    % Log-verosimiglianza dei campioni registrati per una data Qn continua.
    % Usa l'equivalente esatto a campioni del modello continuo.
    n  = size(Aa, 1);
    Ad = sysd.A;  Bd = sysd.B;  Cd = sysd.C;
    Qd = qd_van_loan(Aa, Qn, Ts);          % covarianza per campione equivalente a Qn
    [M, P] = dlqe(Ad, eye(n), Cd, Qd, R);  % M: guadagno, P: covarianza a priori
    S = Cd*P*Cd' + R;                      % varianza teorica dell'innovazione

    % Predittore: stato = x_hat a priori, ingressi = [u; y], uscita = innovazione
    Ap = Ad*(eye(n) - M*Cd);
    Bp = [Bd, Ad*M];
    e  = lsim(ss(Ap, Bp, -Cd, [0, 1], Ts), [u, y], t - t(1), x0);

    em   = e(mask);
    logL = -0.5*sum(log(2*pi*S) + em.^2/S);
end

function Qd = qd_van_loan(Aa, Qn, Ts)
    % Rumore di processo continuo (densita' Qn) integrato su un passo Ts
    n  = size(Aa, 1);
    F  = expm([-Aa, Qn; zeros(n), Aa'] * Ts);
    Qd = F(n+1:end, n+1:end)' * F(1:n, n+1:end);
    Qd = (Qd + Qd')/2;                     % simmetrizza (errori numerici)
end

function v = meno_logL(p, sysd, Aa, R, Ts, u, y, t, x0, mask)
    % -logL in funzione di p = [log10(Q_omega), log10(Q_d)], per fminsearch
    try
        v = -log_verosimiglianza(sysd, Aa, diag([0, 10^p(1), 10^p(2)]), R, Ts, u, y, t, x0, mask);
    catch
        v = Inf;                           % Riccati non risolvibile: punto scartato
    end
end