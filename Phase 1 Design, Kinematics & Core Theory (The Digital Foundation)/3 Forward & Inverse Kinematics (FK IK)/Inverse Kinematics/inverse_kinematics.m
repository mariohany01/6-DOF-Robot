%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

% Define DH parameters in table format [theta_h, d, a, alpha] (in radians)
a1 = 37.5;
a2 = 160;
a3 = 15;
d1 = 135.80;
d2 = 138.1;
d3 = 28.2;

dh_table = [
    pi/2,        d1,   a1,   pi/2;   % Joint 1
    pi/2,     0,    a2,   -pi;      % Joint 2
    pi/2,        0,    a3,   -pi/2;   % Joint 3
    pi/2,        d2,   0,    pi/2;  % Joint 4
    pi/2,        0,    0,    -pi/2;   % Joint 5
    pi/2,        d3,   0,    0       % Joint 6
];


% Extract parameters from dh_table
theta_h = dh_table(:, 1);  % theta_h for each joint
d = [dh_table(1,2), 0, 0, dh_table(4,2), 0, dh_table(6,2)];  % d1, d2=0, d3=0, d4, d5=0, d6
a = [dh_table(1,3), dh_table(2,3), dh_table(3,3), 0, 0, 0];    % a1, a2, a3, a4=0, a5=0, a6=0
alpha = dh_table(:, 4);  % alpha for each joint

% Inverse Kinematics for 6-DOF Robot with Offset and Spherical Wrist
% This is a standalone MATLAB script. Define your 4x4 transformation matrix T below.
% Example T matrix (replace with your actual end-effector pose):
T = [0, 0, -1, -28.2;  % Example: Identity rotation, position [1000, 500, 1200] mm
     -1, 0, 0, 52.5;
     0, 1, 0, 157.7;
     0, 0, 0, 1];

% Extract position and rotation from T
p = T(1:3, 4);  % [px, py, pz]
R = T(1:3, 1:3);  % Rotation matrix
ax = R(1,3); ay = R(2,3); az = R(3,3);  % a vector (z-axis of end-effector)

% Step 1: Compute θ1 (Equation 1)
theta1_v = atan2(p(2) - d(6)*ay, p(1) - d(6)*ax);
theta1_i = theta1_v + theta_h(1);

% Step 2: Compute intermediate points
P01 = [a(1)*cos(theta1_i), a(1)*sin(theta1_i), d(1)];
P04 = [p(1) - d(6)*ax, p(2) - d(6)*ay, p(3) - d(6)*az];
P14 = P04 - P01;
P14L = norm(P14);
l1 = sqrt(a(3)^2 + d(4)^2);

% Step 3: Compute θ3 (Equation 2) - Fixed: removed -pi, changed zeta
zeta = atan2(a(3), d(4));  % Fixed: atan2(a(3), d(4)) instead of atan2(d(4), a(3))
phi = acos((l1^2 + a(2)^2 - P14L^2) / (2 * l1 * a(2)));
theta3_v = phi - zeta;  % Fixed: removed -pi
theta3_i = theta3_v + theta_h(3);

% Step 4: Compute θ2 (Equation 3) - Fixed: changed beta2 and adjustment
beta1 = atan2(P14(3), sqrt(P14(1)^2 + P14(2)^2));
beta2 = acos((a(2)^2 + P14L^2 - l1^2) / (2 * a(2) * P14L));  % Fixed: + P14L^2 instead of - P14L^2
theta2_v = beta1 + beta2 - pi;  % Fixed: -pi instead of -pi/2
theta2_i = theta2_v + theta_h(2);

% Step 5: Compute θ5 (Equation 4)
R3z = [cos(theta1_i)*sin(theta2_i + theta3_i), sin(theta1_i)*sin(theta2_i + theta3_i), -cos(theta2_i + theta3_i)];
R6z = [ax, ay, az];
theta5_v = acos(dot(R6z, R3z));
theta5_i = theta5_v + theta_h(5);

% Step 6: Compute A13 and A46
% Compute A1, A2, A3 matrices (simplified)
A1 = [cos(theta1_i), 0, sin(theta1_i), a(1)*cos(theta1_i);
      sin(theta1_i), 0, -cos(theta1_i), a(1)*sin(theta1_i);
      0, 1, 0, d(1);
      0, 0, 0, 1];
A2 = [cos(theta2_i), -sin(theta2_i), 0, a(2)*cos(theta2_i);
      sin(theta2_i), cos(theta2_i), 0, a(2)*sin(theta2_i);
      0, 0, 1, 0;
      0, 0, 0, 1];
A3 = [cos(theta3_i), 0, sin(theta3_i), a(3)*cos(theta3_i);
      sin(theta3_i), 0, -cos(theta3_i), a(3)*sin(theta3_i);
      0, 1, 0, 0;
      0, 0, 0, 1];
A13 = A1 * A2 * A3;
A13_inv = inv(A13);
A46 = A13_inv * T;

% Step 7: Compute θ4 and θ6 (Equations 5 and 6)
Hx = A46(1,4); Hy = A46(2,4);
theta4_v = atan2(Hy, Hx);
theta4_i = theta4_v + theta_h(4);

Ez = A46(3,1); Fz = A46(3,2);
theta6_v = atan2(Fz, Ez);
theta6_i = theta6_v + theta_h(6);

% Collect thetas in degrees
thetas = [theta1_i, theta2_i, theta3_i, theta4_i, theta5_i, theta6_i] * 180 / pi;

% Display results
fprintf('Computed Joint Angles (degrees):\n');
fprintf('θ1: %.2f\n', thetas(1));
fprintf('θ2: %.2f\n', thetas(2));
fprintf('θ3: %.2f\n', thetas(3));
fprintf('θ4: %.2f\n', thetas(4));
fprintf('θ5: %.2f\n', thetas(5));
fprintf('θ6: %.2f\n', thetas(6));