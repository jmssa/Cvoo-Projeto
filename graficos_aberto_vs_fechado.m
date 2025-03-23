function graficos_aberto_vs_fechado(bb_o,bb_s,p_o,p_s,r_o,r_s,phi_o,phi_s,deltaa_s,deltar_s, t_o,t_s)


figure;
tiledlayout(4,1, 'TileSpacing', 'compact');

nexttile;
plot(t_o, rad2deg(bb_o), 'b', t_s, rad2deg(bb_s), 'r', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de derrapagem', 'FontSize', 14);
legend({'open loop', 'closed loop'}, 'Location', 'best');
grid on;

nexttile;
plot(t_o, rad2deg(p_o), 'b', t_s, rad2deg(p_s), 'r', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Velocidade angular (°/s)', 'FontSize', 14);
title('Razão de rolamento', 'FontSize', 14);
legend({'open loop', 'closed loop'}, 'Location', 'best');
grid on;


nexttile;
plot(t_o, rad2deg(r_o), 'b', t_s, rad2deg(r_s), 'r', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Velocidade angular (°/s)', 'FontSize', 14);
title('Razão de guinada', 'FontSize', 14);
legend({'open loop', 'closed loop'}, 'Location', 'best');
grid on;


nexttile;
plot(t_o, rad2deg(phi_o), 'b', t_s, rad2deg(phi_s), 'r', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de guinada', 'FontSize', 14);
legend({'open loop', 'closed loop'}, 'Location', 'best');
grid on;

figure;
tiledlayout(2,1, 'TileSpacing', 'compact');

nexttile;
plot(t_s, rad2deg(deltaa_s), 'b')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('$\delta_a$','Interpreter', 'Latex', 'FontSize', 14);
grid on;

nexttile;
plot(t_s, rad2deg(deltar_s), 'b')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('$\delta_r$','Interpreter', 'Latex', 'FontSize', 14);
grid on;


da_max = rad2deg(max(deltaa_s))
da_min = rad2deg(min(deltaa_s))
dr_max = rad2deg(max(deltar_s))
dr_min = rad2deg(min(deltar_s))
bb_max = rad2deg(max(bb_s))
bb_min = rad2deg(min(bb_s))



end