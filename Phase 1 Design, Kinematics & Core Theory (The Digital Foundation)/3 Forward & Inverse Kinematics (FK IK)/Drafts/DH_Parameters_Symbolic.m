% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

% --- SYMBOLIC ANALYSIS ---
fprintf('--- Symbolic Forward Kinematics ---\n\n');

% Define symbolic variables for joints and parameters
syms theta_01_rad theta_02_rad theta_03_rad theta_04_rad theta_05_rad theta_06_rad % Joint variables
syms a1 a2 a3 d1 d2 d3   % Link parameters

L_sym(1) = Link([theta_01_rad          d1      a1       pi/2]);        % R
L_sym(2) = Link([theta_02_rad+pi/2     0         a2       -pi]);         % R
L_sym(3) = Link([theta_03_rad          0         a3       -pi/2]);       % Ghayar L3 le d3
L_sym(4) = Link([theta_04_rad          d2      0          pi/2]);        % R
L_sym(5) = Link([theta_05_rad          0         0          -pi/2]);       % R
L_sym(6) = Link([theta_06_rad          d3      0           0]);          % R

% Create the symbolic robot model
Rob_sym = SerialLink(L_sym, 'name', 'Symbolic RRRRRR');

% Define the symbolic joint variable vector
q_sym = [theta_01_rad theta_02_rad theta_03_rad theta_04_rad theta_05_rad theta_06_rad];

% Calculate the symbolic forward kinematics transformation matrix
T_sym = Rob_sym.fkine(q_sym);

% Extract the 4x4 matrix from the symbolic SE3 object before displaying
%disp('Not Simple Symbolic Transformation Matrix (T):');
T_matrix_sym = T_sym.T;

% Display the simplified symbolic transformation matrix
disp('Symbolic Transformation Matrix (T):');
disp(simplify(T_matrix_sym));

% --- STORE THE 4x4 MATRIX ELEMENTS IN VARIABLES ---
% Extract and store each element of the transformation matrix
nxf = T_matrix_sym(1,1);
oxf = T_matrix_sym(1,2);
axf = T_matrix_sym(1,3);
pxf = T_matrix_sym(1,4);

nyf = T_matrix_sym(2,1);
oyf = T_matrix_sym(2,2);
ayf = T_matrix_sym(2,3);
pyf = T_matrix_sym(2,4);

nzf = T_matrix_sym(3,1);
ozf = T_matrix_sym(3,2);
azf = T_matrix_sym(3,3);
pzf = T_matrix_sym(3,4);

z0ef = atan2(ayf, axf);
y1ef = atan2(sqrt(1 - azf^2), azf);
z2ef = atan2(ozf, -nzf);

% The last row is [0, 0, 0, 1], but since it's symbolic, we can confirm:
% Note: T_matrix_sym(4,1) to T_matrix_sym(4,4) should be [0, 0, 0, 1]

% Optional: Display the stored variables for verification
fprintf('\n--- Stored Variables ---\n');
fprintf('nxf = '); disp(nxf);
fprintf('nyf = '); disp(nyf);
fprintf('nzf = '); disp(nzf);
fprintf('oxf = '); disp(oxf);
fprintf('oyf = '); disp(oyf);
fprintf('ozf = '); disp(ozf);
fprintf('axf = '); disp(axf);
fprintf('ayf = '); disp(ayf);
fprintf('azf = '); disp(azf);
fprintf('pxf = '); disp(pxf);
fprintf('pyf = '); disp(pyf);
fprintf('pzf = '); disp(pzf);

fprintf('z0ef = '); disp(z0ef);
fprintf('y1ef = '); disp(y1ef);
fprintf('z2ef = '); disp(z2ef);