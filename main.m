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

[A,B,x0, u0] = init();

%% Simulacao com recurso ao Simulink do sistema em anel aberto




% Definicao das saidas do sistema
C = diag([1,1,1,1]);
D = zeros(4,2);

tsim = 40;
open("UAV3.slx");
uav3 = sim("UAV3.slx");


%% SAE para a derrapagem com recurso ao "yaw damper"

K = k_finder(A,B);

%%

open("UAV3SAE.slx");
uav3SAE = sim("UAV3SAE.slx");

%%

graficos_aberto_vs_fechado(bb_o,bb_s,p_o,p_s,r_o,r_s,phi_o,phi_s,deltaa_s,deltar_s, t_o,t_s)

% Analise dos valores maximos e minimos permitidos

da_max = rad2deg(max(deltaa_s))
da_min = rad2deg(min(deltaa_s))
dr_max = rad2deg(max(deltar_s))
dr_min = rad2deg(min(deltar_s))
bb_max = rad2deg(max(bb_s))
bb_min = rad2deg(min(bb_s))

