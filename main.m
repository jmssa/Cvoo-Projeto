%% UAV - Flight condition 3
%
% Codigo para o projeto de Controlo de Voo 2024/2025
% Autores:
% * Joao Santos 106280
% * Francisco Garcia 106385
% * Ruben Bernardino 106571
 
clear;
clc;
close all;
%% condições

[A,B, x0, u0] = init();

%% Simulacao com recurso ao Simulink do sistema em anel aberto

% Definicao das saidas do sistema
C = diag([1,1,1,1]);
D = zeros(4,2);

C_lqr = diag([1,1,1,1,1,1]);
D_lqr = zeros(6,2);

tsim = 40;
open("UAV3.slx");
uav3 = sim("UAV3.slx");


%% SAE para a derrapagem com recurso ao "yaw damper"

K = k_finder(A,B);

%%

open("UAV3SAE.slx");
uav3SAE = sim("UAV3SAE.slx");

graficos_aberto_vs_fechado(bb_o,bb_s,p_o,p_s,r_o,r_s,phi_o,phi_s,deltaa_s,deltar_s, t_o,t_s)

%% Simulacao do Controlo de Atitute

%meter as cenas inicias para só correr esta seccção de código
clc
clear 
close all;

tsim = 40;
[A,B, x0, u0] = init();


[A,B,x0,dim] = new_A_B_lambda(A,B);

%ver se o sistema é  controlável
if dim == rank(ctrb(A,B))
    disp("O novo sistema é controlável");
else
    disp("O sistema não é controlável");
end

%assumir acesso a todos os estados
C = diag([1,1,1,1,1]);

%sem respostas instantaneas
D = zeros(5,2);

Q = diag([10,1,1,1,30]);
R = diag([1,1]);

K = lqr(A,B,Q,R);

%referencia
r = [deg2rad(10), deg2rad(20)];

open("UAV3atitude.slx");
sim("UAV3atitude.slx");

graficos_gerais(t_lqr,bb_lqr,lambda_lqr,p_lqr,r_lqr,phi_lqr,deltaa_lqr,deltar_lqr);

confirmar_valores_finais(r,bb_lqr,lambda_lqr);



