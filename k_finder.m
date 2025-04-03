function K = k_finder(A,B)

%output ser apenas o r;
C_temp = [0,0,1,0];
D_temp = 0;

%boundary conditions
chi = 0.6;
angle = asin(chi);
wn = 1;

G = ss(A,B(:,2),C_temp,D_temp)

figure
rlocus(G)
xlim([-30,1])
hold on      

% Representar as restricoes no LGR
L = linspace(0,100,9999);
x1 = L*cos((pi/2) + angle);
y1 = L*sin((pi/2) + angle);

x2 = L*cos(-(pi/2)-angle);
y2 = L*sin(-(pi/2)-angle);

theta_temp = linspace(pi/2,3*pi/2, 9999);
x3 = wn*cos(theta_temp);
y3 = wn*sin(theta_temp);

plot(x1,y1, 'b');
plot(x2,y2, 'b');
plot(x3,y3, 'b');

hold off;

% A partir da analise do lugar geometrico das raizes, definir o K

K = [0,0,0,0;
     0,0, 0.314,0];

%wn>1
%chi>0.19
%polos reais
%margem para n ter oscilações

damp(A-B*K);

end