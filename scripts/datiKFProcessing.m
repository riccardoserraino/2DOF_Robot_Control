t     = out.dati.Time;
u     = out.dati.Data(:,2);
theta = out.dati.Data(:,3);

% --- Se invece hai salvato con To Host File (righe [t; u; theta]):
% S = load('prova_motore1.mat');  M = S.(char(fieldnames(S)));
% if size(M,1) == 3, M = M'; end
% t = M(:,1);  u = M(:,2);  theta = M(:,3);

% --- Controlli veloci prima del KF
fprintf('Ts = %.4g s, durata = %.1f s, campioni = %d\n', t(2)-t(1), t(end)-t(1), numel(t));
fprintf('u in [%.2f, %.2f] V, theta in [%.1f, %.1f] gradi\n', ...
        min(u), max(u), min(theta)*180/pi, max(theta)*180/pi);
figure; subplot(2,1,1); plot(t, u); ylabel('u [V]');
subplot(2,1,2); plot(t, theta*180/pi); ylabel('\theta [gradi]'); xlabel('t [s]');

save('dati_prova_motore1.mat', 't', 'u', 'theta');   % salvali, cosi' non li perdi

% --- Modello da provare, poi lancia lo script
K = k_3165_load;  tau = tau_3165_load;      % i tuoi valori identificati
run('KF.m')