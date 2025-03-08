function [A,B] = init()

%% Dados da aeronave

u0 = 54.4; %kn
u0 = u0*0.51444; %m/s


aa0 = -3.65; %deg
aa0 = deg2rad(aa0); %rad

gg0 = 0; %deg/rad

tt0 = gg0 + aa0; %rad


w0 = tan(aa0)*u0; %m/s

% Inertical data
[Ix, Iy, Iz, Ixz] = deal(1.548, 2.841, 3.828, 0.1); %kg/m^2
g = 9.81; %m/s^2


% Derivatives (no units or SI units ):
ybb = -0.1888; lbb = -110.6918; nbb = 38.3679;
yp = 0.0000; lp = -23.7667; np = -1.9032; 
yr = 0.0000; lr = 8.0007; nr = -0.4885;

Lda = -130.897; Nda = -0.000; Ydr = -0.008; Ldr = -0.000; Ndr = 39.434;

%% Modelo em Espaco de Estados da aeronave

% Variaveis que nao estao definidas no enunciado
Yda = 0;

% Vetor de estado e entradas a considerar
%   X_lat = [bb; p; r; phi];
%   U_lat = [da; dr];

% Calculo das derivadas adicionais a utilizar
llbb = lbb + (Ixz/Ix)*nbb;
llp = lp + (Ixz/Ix)*np;
llr = lr + (Ixz/Ix)*nr;
Llda = Lda + (Ixz/Ix)*Nda;
Lldr = Ldr + (Ixz/Ix)*Ndr;

nlbb = nbb + (Ixz/Iz)*lbb;
nlp = np + (Ixz/Iz)*lp;
nlr = nr + (Ixz/Iz)*lr;
Nlda = Nda + (Ixz/Iz)*Lda;
Nldr = Ndr + (Ixz/Iz)*Ldr;

% Definicao das matrizes da dinamica e das entradas
A = [ybb, yp+(w0/u0), yr-(u0/u0), g*cos(tt0)/u0;
        llbb, llp, llr, 0;
        nlbb, nlp, nlr, 0;
        0, 1, tan(tt0), 0];

B = [Yda, Ydr;
         Llda, Lldr;
         Nlda, Nldr;
         0  , 0  ];

