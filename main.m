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

[A,B, A_lqr, B_lqr, x0, u0] = init();

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

%open("UAV3SAE.slx");
%uav3SAE = sim("UAV3SAE.slx");

%graficos_aberto_vs_fechado(bb_o,bb_s,p_o,p_s,r_o,r_s,phi_o,phi_s,deltaa_s,deltar_s, t_o,t_s)

%% Simulacao do Controlo de Atitute

Q = diag([1,1,1,1,1,1]);
R = diag([1,1]);
K_lqr = lqr(A_lqr,B_lqr,Q,R,0)

open("UAV3atitude.slx");
UAV3atitude = sim("UAV3atitude.slx");

%graficos_aberto_vs_fechado(bb_o,bb_lqr,p_o,p_lqr,r_o,r_lqr,phi_o,phi_lqr,deltaa_lqr,deltar_lqr, t_o,t_lqr)


%% Analise dos valores maximos e minimos permitidos

da_max = rad2deg(max(deltaa_s))
da_min = rad2deg(min(deltaa_s))
dr_max = rad2deg(max(deltar_s))
dr_min = rad2deg(min(deltar_s))
bb_max = rad2deg(max(bb_s))
bb_min = rad2deg(min(bb_s))
