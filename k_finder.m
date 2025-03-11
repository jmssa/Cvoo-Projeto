function K = k_finder(A,B)

%output ser apenas o r;
C_temp = [0,0,1,0];
D_temp = 0;

%boundary conditions
chi = 0.19;
angle = asin(chi);
wn = 1;

G = ss(A,B(:,2),C_temp,D_temp);

figure
rlocus(G)
hold on      

L = linspace(0,100,9999);
x1 = L*cos(angle + pi);
y1 = L*sin(angle + pi);

x2 = L*cos(-angle - pi);
y2 = L*sin(-angle - pi);

theta_temp = linspace(pi/2,3*pi/2, 9999);
x3 = wn*cos(theta_temp);
y3 = wn*sin(theta_temp);

plot(x1,y1, 'b');
plot(x2,y2, 'b');
plot(x3,y3, 'b');

hold off;

K = [0,0,0,0;
     0,0, 0.485,0];

%wn>1
%chi>0.19
%polos reais
%margem para n ter oscilações

damp(A-B*K);

end