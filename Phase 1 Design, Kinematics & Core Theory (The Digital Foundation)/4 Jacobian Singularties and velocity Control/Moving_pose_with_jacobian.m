clc;clear;close all;

a1 = 37.5;a2 = 160;a3 = 15;
d1 = 135.80;d2 = 138.1;d3 = 28.2;  


% Link([theta, d, a, alpha])
L(1) = Link([0      d1     a1        pi/2]); 
L(2) = Link([pi/2   0      a2       -pi]);
L(3) = Link([0      0      a3       -pi/2]);
L(4) = Link([0      d2     0        pi/2]);
L(5) = Link([0      0      0        -pi/2]);
L(6) = Link([0      d3     0        0]);

Kuka = SerialLink(L, 'name', 'My_Kuka');
q0 = [0 pi/2 0 0 0 0];
% q = [30*pi/180 -45*pi/180 0.2 60*pi/180 20*pi/180 90*pi/180];
Kuka.plot(q0, 'workspace', [-400 400 -400 400 -1 400]);
T0 = Kuka.fkine(q0);

disp("Fist Joint Vector (q0):")
disp(q0);
disp("Fist transformation matrix (T0):")
T0_d = double(T0);
disp(T0_d);
disp("First transformation Translation (T1):")
T0_translation = transl(T0);
disp(T0_translation);
disp("================================")


dq1 = 1e-6;
q1=q0+[1e-6 0 0 0 0 0];
T1 = Kuka.fkine(q1);

disp("Second Joint Vector(q1):")
disp(q1);
disp("Second transformation matrix (T1):")
T1_d = double(T1);
disp(T1_d);
disp("Second transformation Translation (T1):")
T1_translation = transl(T1);
disp(T1_translation);

disp("================================")
disp("Calculate the whole drivative")
dT_dq1=(T1-T0)/dq1;
disp(dT_dq1)

disp("================================")
disp("Find Translation ")
dt_dq1 = dT_dq1(1:3, 4);
disp("Translation Vector for q1")

disp(dt_dq1)

%% Rotation Part
disp("================================")
disp("Find Rotation ")
dr_dq1=dT_dq1(1:3,1:3);

disp("dR_q1")
disp(dr_dq1);

R=T0_d(1:3,1:3);
disp("Rotation matrix from T0")
disp(R);

S=dr_dq1*inv(R);
disp("S")
disp(S);

omega_Vector=vex(S);
disp("Omega Vector for q1")
disp(omega_Vector);

disp("================================")
disp("j1 Vector ")
j1=[dt_dq1;omega_Vector];
disp(j1);