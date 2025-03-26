function [L] = L_estim(A,C)



G = diag([1,1,1,1,1])*0.01;
Qe = diag([1,1,1,1,1]);
Re = diag([1,1,1]);
L = lqe(A,G,C,Qe,Re);