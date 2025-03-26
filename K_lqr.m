function [K] = K_lqr(A,B)

[A_temp,B_temp,dim_temp] = new_A_B_integrativos(A,B);
Q = diag([50,3,4,1,60,75,75]);
R = diag([40,30]);
K = lqr(A_temp,B_temp,Q,R);