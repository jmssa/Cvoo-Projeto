function grafico_est_vs_real_vs_medido(t, bb_est,bb, lambda_est, lambda, p_est, p, r_est,r, p_lido, r_lido)


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