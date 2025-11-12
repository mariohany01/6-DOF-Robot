%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

%% Links Joints Parameters
% Links Parameters
a1 = 37.2;
a2 = 138.10;
a3 = 28.2;
d1 = 135.80;
d2 = 160;
d3 = 15;

% Define joint angles in degrees (user inputs)
theta_01_deg = 0;     % Joint 1 angle in degrees
theta_02_deg = 90;    % Joint 2 angle in degrees
theta_03_deg = 0;     % Joint 3 angle in degrees
theta_04_deg = 0;     % Joint 4 angle in degrees
theta_05_deg = 0;     % Joint 5 angle in degrees
theta_06_deg = 0;     % Joint 6 angle in degrees

%% Convert joint angles from degrees to radians for use in the robot model
q = [theta_01_deg, theta_02_deg, theta_03_deg, theta_04_deg, theta_05_deg, theta_06_deg] * (pi / 180);

%% Convert joint angles from degrees to radians for use in the robot model
theta_01_rad = q(1);
theta_02_rad = q(2);
theta_03_rad = q(3);
theta_04_rad = q(4);
theta_05_rad = q(5);
theta_06_rad = q(6);

%% DH Parameters

L(1) = Link([theta_01_deg          d1      a1        pi/2]);        % R
L(2) = Link([theta_02_deg+pi/2     0       a2       -pi]);         % R
L(3) = Link([theta_03_deg          0       a3       -pi/2]);       % Ghayar L3 le d3
L(4) = Link([theta_04_deg          d2      0         pi/2]);        % R
L(5) = Link([theta_05_deg          0       0        -pi/2]);       % R
L(6) = Link([theta_06_deg          d3      0           0]);          % R

%% Plot Robot
disp('Robot plot generated successfully.');
Rob = SerialLink(L, 'name', 'RRRRRR')
Rob.plot(q, 'workspace', [-400 400 -400 400 -1 500]);

%% Get Forward Kinematics
T = Rob.fkine(q);
%remove the small values that equals to zero
threshold = 1e-10;
T_matrix = double(T);                       % Convert to double matrix if needed
T_matrix(abs(T_matrix) < threshold) = 0;
disp(T_matrix);                             %display the Transformation Matrix From the Toolbox


%% Trying the equations
pxf = a1*cos(theta_01_rad) + a2*cos(theta_01_rad)*cos(theta_02_rad) + d2*sin(theta_02_rad - theta_03_rad)*cos(theta_01_rad) + a3*cos(theta_01_rad)*cos(theta_02_rad)*cos(theta_03_rad) + a3*cos(theta_01_rad)*sin(theta_02_rad)*sin(theta_03_rad) - d3*sin(theta_01_rad)*sin(theta_04_rad)*sin(theta_05_rad) + d3*sin(theta_02_rad - theta_03_rad)*cos(theta_01_rad)*cos(theta_05_rad) - d3*cos(theta_01_rad)*cos(theta_02_rad)*cos(theta_03_rad)*cos(theta_04_rad)*sin(theta_05_rad) - d3*cos(theta_01_rad)*cos(theta_04_rad)*sin(theta_02_rad)*sin(theta_03_rad)*sin(theta_05_rad);
pyf = a1*sin(theta_01_rad) + a2*cos(theta_02_rad)*sin(theta_01_rad) + d2*sin(theta_02_rad - theta_03_rad)*sin(theta_01_rad) + a3*cos(theta_02_rad)*cos(theta_03_rad)*sin(theta_01_rad) + d3*cos(theta_01_rad)*sin(theta_04_rad)*sin(theta_05_rad) + a3*sin(theta_01_rad)*sin(theta_02_rad)*sin(theta_03_rad) + d3*sin(theta_02_rad - theta_03_rad)*cos(theta_05_rad)*sin(theta_01_rad) - d3*cos(theta_02_rad)*cos(theta_03_rad)*cos(theta_04_rad)*sin(theta_01_rad)*sin(theta_05_rad) - d3*cos(theta_04_rad)*sin(theta_01_rad)*sin(theta_02_rad)*sin(theta_03_rad)*sin(theta_05_rad);
pzf = d1 + a2*sin(theta_02_rad) - d2*cos(theta_02_rad - theta_03_rad) + a3*sin(theta_02_rad - theta_03_rad) - (d3*sin(theta_02_rad - theta_03_rad)*sin(theta_04_rad + theta_05_rad))/2 - d3*cos(theta_02_rad - theta_03_rad)*cos(theta_05_rad) + (d3*sin(theta_02_rad - theta_03_rad)*sin(theta_04_rad - theta_05_rad))/2;
disp("Position Comparison (Toolbox vs Manual):");

pos_toolbox = transl(T_matrix);

fprintf("Equations : %.4f , %.4f , %.4f\n",pxf,pyf,pzf)
fprintf('Toolbox   : %.4f , %.4f , %.4f\n', pos_toolbox(1), pos_toolbox(2), pos_toolbox(3));
 

%% Finding Angles
