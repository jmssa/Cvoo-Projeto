function [A,B, dim]=new_A_B_integrativos(A,B)

%estados atuais
% X = [bb; p; r; phi; lambda];

% definir estado novo w = int(Cx - r)
% isto é w. = Cx - r
% se fizermos w convergir então w. = 0 => erro = 0
%
% Vamos ter a nova formação
% x'. = A'x' + Bu - R
% y' = C'x'
%
%
% Onde 
% 
% x' = [x w]
% 
% A' = [A 0
%       C 0]
% 
% B' = [B
%       0]
% 
% R = [0
%      r]
%
% C' = [C 0] (vamos meter na mesma diagonal tudo a 1 para não haver
% problemas com estimação
%
% Damos design do controlador com esta nova matriz

%% A
new_line_bb = [1, 0, 0, 0, 0];
new_line_lambda = [0, 0, 0, 0, 1];

zeros = [0,0,0,0,0,0,0]';

A = [A;new_line_bb; new_line_lambda];

A = [A, zeros, zeros];

%% B

new_line_bb = [0,0];
new_line_lambda = [0,0];

B = [B; new_line_bb; new_line_lambda];


dim = 7;




