function graficos_formacao(E_leader, N_leader, E_follow, N_follow, x_diff, y_diff)

figure;
tiledlayout(1,2, 'TileSpacing', 'compact');

nexttile;
plot(E_leader.signals.values, N_leader.signals.values, 'r', E_follow.signals.values, N_follow.signals.values, 'blue', 'LineWidth', 1.5);
xlabel('Este (m)', 'FontSize', 14);
ylabel('Norte (m)', 'FontSize', 14);
title('Posição das aeronaves no referencial fixo', 'FontSize', 14);
legend('Líder', 'Seguidora');
grid on;

nexttile
plot(x_diff.signals.values, y_diff.signals.values, 'Color', [0 0.7 0], 'LineWidth', 1.5);
xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);
title('Posição da líder no referencial da seguidora', 'FontSize', 14);
grid on;




figure;
hold on;
x0 = min(y_diff.signals.values);
y0 = min(x_diff.signals.values);
plot3(x_diff.signals.values, x_diff.time, y_diff.signals.values, 'Color', [0 0.7 0], 'LineWidth', 1.5);
plot3(x_diff.signals.values, x_diff.time, x0*ones(size(x_diff.time)),  '--', 'Color', [0 0.9 0]);
plot3(y0*ones(size(x_diff.time)), x_diff.time, y_diff.signals.values, '--', 'Color', [0 0.9 0]);
grid on
xlabel('X (m)')
ylabel('Tempo (s)')
zlabel('Y (m)')
title('Aeronave líder no referencial da seguidora ao longo do tempo')
axis tight
view(45, 30)
hold off



end


