%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures


% Links Parameters
a1 = 175;
a2 = 890;
a3 = 50;
d1 = 575;
d2 = 1035;
d3 = 185;  


% Links Parameters
%a1 = 37.5;
%a2 = 160;
%a3 = 15;
%d1 = 135.80;
%d2 = 138.1;
%d3 = 28.2;   

%%
% Link([theta, d, a, alpha])

L(1) = Link([0      d1     a1        pi/2]); 
L(2) = Link([pi/2   0      a2       -pi]);   % Note: pi/2 is a constant offset here
L(3) = Link([0      0      a3       -pi/2]); % REVOLUTE. (If you want prismatic, add 1 at the end)
L(4) = Link([0      d2     0        pi/2]);
L(5) = Link([0      0      0        -pi/2]);
L(6) = Link([0      d3     0        0]);     % d3 is the offset here

Rob = SerialLink(L, 'name', 'RRRRR');
q = [0 pi/2 0 0 0 0];
% q = [30*pi/180 -45*pi/180 0.2 60*pi/180 20*pi/180 90*pi/180];
Rob.plot(q, 'workspace', [-2000 2000 -2000 2000 -1 2000]);
T = Rob.fkine(q);

% Eliminate very small values from T (set values below threshold to 0)
%threshold = 1e-10;
%T_matrix = double(T);  % Convert to double matrix if needed
%T_matrix(abs(T_matrix) < threshold) = 0;
disp(T);

% Sliders
 Rob.teach;


eul_zyz_rad = tr2eul(T);
% ---------------------

disp('ZY''Z" Euler Angles (Radians):');
disp(eul_zyz_rad);

% Optional: Display in degrees
eul_zyz_deg = eul_zyz_rad * 180/pi;
disp('ZY''Z" Euler Angles (Degrees):');
disp(eul_zyz_deg);

% ... (your Rob.teach line) ...