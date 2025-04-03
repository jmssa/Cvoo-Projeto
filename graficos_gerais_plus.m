function graficos_gerais_plus(t,bb_ref, lambda_ref,bb,lambda,p,r,phi, deltaa, deltar, xi)

figure;
tiledlayout(2,1, 'TileSpacing', 'compact');

nexttile;
plot( t, rad2deg(bb_ref), 'black',t, rad2deg(bb),'r', 'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de derrapagem', 'FontSize', 14);
legend('Valor de Referência', 'Resposta');
grid on;

nexttile;
plot(t, rad2deg(lambda_ref),'black', xi.time, rad2deg(xi.signals.values), 'r',  'LineWidth', 1.5);
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('Ângulo de rumo', 'FontSize', 14);
legend('Valor de Referência','Resposta Xi');
grid on;

figure
tiledlayout(2,1, 'TileSpacing', 'compact');

nexttile;
plot(t, rad2deg(deltaa), 'b')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('$\delta_a$','Interpreter', 'Latex', 'FontSize', 14);
grid on;

nexttile;
plot(t, rad2deg(deltar), 'b')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('$\delta_r$','Interpreter', 'Latex', 'FontSize', 14);
grid on;

figure
tiledlayout(3,1, 'TileSpacing', 'compact');

nexttile;
plot(t, p, 'r')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Razão (°/s)', 'FontSize', 14);
title('p','Interpreter', 'Latex', 'FontSize', 14);
grid on;

nexttile;
plot(t, r, 'r')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Razão (°/s)', 'FontSize', 14);
title('r','Interpreter', 'Latex', 'FontSize', 14);
grid on;

nexttile;
plot(t, rad2deg(phi), 'r')
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Ângulo (°)', 'FontSize', 14);
title('$\phi$','Interpreter', 'Latex', 'FontSize', 14);
grid on;

% da_max = rad2deg(max(deltaa))
% da_min = rad2deg(min(deltaa))
% dr_max = rad2deg(max(deltar))
% dr_min = rad2deg(min(deltar))