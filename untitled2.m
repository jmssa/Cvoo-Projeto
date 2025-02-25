%% UAV - Flight condition 3
%
% Codigo para o projeto de Controlo de Voo 2024/2025
% Autores:
% * Joao Santos 106280
% * Francisco Garcia 106385
% * Ruben Bernardino 106571

%% Dados da aeronave

h = 500; %m 
aa0 = -3.65; %deg 
gg0 = 0; %deg
tt0 = gg0 + aa0; %deg

u0 = 54.4;
u0 = u0*0.51444; % Ja em metros por segundo
flaps = 0; %deg

w0 = tan(aa0)*u0;

% Inputs
th0 = 83; %%
de0 = 5.07; %deg
da0 = 0.34; %deg 
dr0 = -0.01; %deg
Teng = 0.14; %sec
demax = 30; %deg
demin = -30; %deg
damax = 30; %deg
drmax = 30; %deg 
flapmax = 40; %deg

% Inertical data
m =26.5; %kg;
[Ix, Iy, Iz, Ixz] = deal(1.548, 2.841, 3.828, 0.1); %kg/m^2
g = 9.81; %m/s^2

% Wing data
S = 0.90; %m^2
b = 3.000; %m
c = 0.300; %m
aamax = 18.00; %deg

% Derivatives (no units or SI units ):
xu = -0.1240; xw = 0.5242; zu = -0.7717; zw = -3.5322; zwp = -0.0026;
zq = -1.8328; mu = 0.0000; mw = -2.1311; mq = -3.6798; mwp = -0.0861;

ybb = -0.1888; lbb = -110.6918; nbb = 38.3679;
yp = 0.0000; lp = -23.7667; np = -1.9032; 
yr = 0.0000; lr = 8.0007; nr = -0.4885;

xde = -0.000; zde = 1.815; mde = -59.208;
xdsp = -17.106; zdsp = 5.132; mdsp = 0.000;
xdt = 2.642; zdt = 0.000; mdt = 0.000;

Lda = -130.897; Nda = -0.000; Ydr = -0.008; Ldr = -0.000; Ndr = 39.434;

%% Modelo em Espaco de Estados da aeronave
%X_lat = [bb; p; r; phi];

%U_lat = [da; dr];

A_lat = [ybb, yp+w0, yr-u0, g*cos(tt0);
        lbb, lp, lr, 0;
        nbb, np, nr, 0;
        0, 1, tan(tt0), 0];


B_lat = [];

damp(A_lat)

