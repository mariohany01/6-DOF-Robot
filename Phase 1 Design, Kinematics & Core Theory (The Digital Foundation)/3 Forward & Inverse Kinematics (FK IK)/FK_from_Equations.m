clc;close;clear;

a1=37.510;
a2=138.10; 
a3=28.20;
d1= 135.80;
d2= 160.00;
d3= 15.00;

%%
%My robot
% Link([theta, d, a, alpha])
L(1) = Link([0     d1       a1       pi/2]);            
L(2) = Link([pi/2     0    a2      -pi]);           
L(3) = Link([0     0    a3     -pi/2]);       
L(4) = Link([0     d2       0       pi/2]);            
L(5) = Link([0     0        0       -pi/2]);           
L(6) = Link([0     d3  0       0]);   
        
Rob = SerialLink(L, 'name', 'RRRRR');
q=[0 90*(pi / 180) 0 0 0 0];
Rob.plot(q, 'workspace', [-400 400 -400 400 -1 500]);
T = Rob.fkine(q); disp(T);
% Sliders
%Rob.teach;

