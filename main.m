%% UAV - Flight condition 3
%
% Codigo para o projeto de Controlo de Voo 2024/2025
% Autores:
% * Joao Santos 106280
% * Francisco Garcia 106385
% * Ruben Bernardino 106571
 
clear;
clc;

%% condições

[A,B,x0, u0] = init();

%% Simulacao com recurso ao Simulink do sistema em anel aberto




% Definicao das saidas do sistema
C = diag([1,1,1,1]);
D = zeros(4,2);

tsim = 40;
open("UAV3.slx");
uav3 = sim("UAV3.slx");


%% SAE para a derrapagem com recurso ao "yaw damper"

% O valor de K foi tirado a olho do lugar geométrico das raízes
K = [0, 0, 0,     0;
     0, 0, 0.229, 0];
    
damp(A-B*K)

open("UAV3SAE.slx");
uav3SAE = sim("UAV3SAE.slx");

%%





figure
hold on
plot(uav3.bb.time, uav3.bb.signals.values);
plot(uav3SAE.bb.time, uav3SAE.bb.signals.values);
title("Resposta do angulo de derrapagem.");
legend("Sem SAE", "Com SAE");
figure
hold on
plot(uav3.p.time, uav3.p.signals.values);
plot(uav3SAE.p.time, uav3SAE.p.signals.values);
title("Resposta da razao de rolamento.");
legend("Sem SAE", "Com SAE");
figure
hold on
plot(uav3.r.time, uav3.r.signals.values);
plot(uav3SAE.r.time, uav3SAE.r.signals.values);
title("Resposta da razao de guinada.");
legend("Sem SAE", "Com SAE");
figure
hold on
plot(uav3.phi.time, uav3.phi.signals.values);
plot(uav3SAE.phi.time, uav3SAE.phi.signals.values);
title("Resposta do angulo de rolamento.");
legend("Sem SAE", "Com SAE");
hold off

da_max = rad2deg(max(uav3SAE.deltaa.signals.values))
dr_max = rad2deg(max(uav3SAE.deltar.signals.values))