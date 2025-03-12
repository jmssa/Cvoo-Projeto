function [A,B, dim] = new_A_B_lambda(A,B)

%estados anteriores
% X = [bb; p; r; phi];

%novos estados
% X = [bb; p; r; phi; lamda];

%lambda_p = bb_p + psi_p

% bb_p = A(1,:) 
% psi_p = (1/cos(tt0)) * r

aa0 = -3.65; %deg
aa0 = deg2rad(aa0); %rad

gg0 = 0; %deg/rad
tt0 = gg0 + aa0; %rad

%% adicionar o lambda no A

%nova linha = A(bb_p) + A(psi_p)
new_line_A = A(1,:) + [0, 0, 1/cos(tt0), 0];

A = [A; new_line_A];

zeros = [0, 0, 0, 0, 0]';

%adicionar a coluna de zeros para o outro estado
A = [A, zeros];

%% adicionar o lambda no B

%nova linha = B(bb_p) + B(psi_p)
new_line_B = B(1,:) + [0,0];

B = [B; new_line_B];

%% nova dim do sistema
dim = 5;

