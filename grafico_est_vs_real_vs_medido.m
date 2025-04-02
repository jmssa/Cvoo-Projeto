function grafico_est_vs_real_vs_medido(t, bb_est,bb, lambda_est, lambda, p_est, p, r_est,r, psi_est, psi, p_lido, r_lido, psi_lido)

%% cenas q estamos a seguir
figure;
tiledlayout(2,1, 'TileSpacing', 'compact');

nexttile;
plot(t, rad2deg(bb),'r', t, rad2deg(bb_est), 'blue', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de derrapagem', 'FontSize', 14);
legend('Valor Estimado','Valor Real');
grid on;

nexttile;
plot(t, rad2deg(lambda), 'r', t, rad2deg(lambda_est),'blue', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de rumo', 'FontSize', 14);
legend('Valor Estimado','Valor Real');
grid on;


%% valores q estamos a medir
figure;
tiledlayout(3,1, 'TileSpacing', 'compact');

nexttile;
plot(t, rad2deg(p_est),'r', t, rad2deg(p), 'blue', t, rad2deg(p_lido), 'green', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Razão (°/s)', 'FontSize', 14);
title('Razão de rolamento', 'FontSize', 14);
legend('Valor Estimado','Valor Real', 'Valor lido');
grid on;


nexttile;
plot(t, rad2deg(r_est),'r', t, rad2deg(r), 'blue', t, rad2deg(r_lido), 'green', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Razão (°/s)', 'FontSize', 14);
title('Razão de guinada', 'FontSize', 14);
legend('Valor Estimado','Valor Real', 'Valor lido');
grid on;

nexttile;
plot(t, rad2deg(psi_est),'r', t, rad2deg(psi), 'blue', t, rad2deg(psi_lido), 'green', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de guinada', 'FontSize', 14);
legend('Valor Estimado','Valor Real', 'Valor lido');
grid on;