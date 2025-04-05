function graficos_formacao(E_leader, N_leader, E_follow, N_follow)

figure;
tiledlayout(2,1, 'TileSpacing', 'compact');

nexttile;
plot(E_leader, N_leader, 'r', E_follow, N_follow, 'blue', 'LineWidth', 1.5);
xlabel('Este (m)', 'FontSize', 14);
ylabel('Norte (m)', 'FontSize', 14);
title('Posição das aeronaves no referencial fixo', 'FontSize', 14);
legend('Líder', 'Seguidora');
grid on;

nexttile;
x = E_leader - E_follow;
y = N_leader - N_follow;
plot(x, y, 'yellow', 'LineWidth', 1.5);
xlabel('Este (m)', 'FontSize', 14);
ylabel('Norte (m)', 'FontSize', 14);
title('Posição da aeronave líder no referencial da seguidora', 'Interpreter', 'Latex', 'FontSize', 14);
grid on;

xlim_max = max(abs(x));
ylim_max = max(abs(y));
lim_max = max([xlim_max, ylim_max]); 
xlim([-lim_max, lim_max]);
ylim([-lim_max, lim_max]);
axis equal;

end


