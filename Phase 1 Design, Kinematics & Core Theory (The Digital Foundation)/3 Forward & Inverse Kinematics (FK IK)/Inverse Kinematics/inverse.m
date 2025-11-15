clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

%% Robot Parameters
a1 = 37.5;
a2 = 160;
a3 = 15;
d1 = 135.80;
d2 = 138.1;
d3 = 28.2;

%% Transformation Matrix
r11= 0;
r12= 0;
r13= 1;
r21= 0;
r22= -1;
r23= 0;
r31= 1;
r32= 0;
r33= 0;
px= 203.8;
py= 0;
pz= 310.8;

Transformation_Matrix = [r11 r12 r13 px;
                        r21 r22 r23 py;
                        r31 r32 r33 pz;
                        0    0   0   1];
P06 = [px; py; pz];
R06 = [r11 r12 r13;
       r21 r22 r23;
       r31 r32 r33];

%% Inverse Kinematics
fprintf('Running 6-DOF Robot Inverse Kinematic Analysis...\n\n');

%% Finding Theta 1
P46 = d3 * [r13; r23; r33];
P04 = P06 - P46;
theta01_deg = atan2d(P04(2), P04(1));
fprintf('Theta 1 = %.2f degrees\n', theta01_deg);

%% Finding Theta 3 (Calculate this BEFORE Theta 2)
% Note: In the paper, theta_1i is already known when calculating theta_3i
theta01_rad = deg2rad(theta01_deg);
P01 = [a1*cosd(theta01_deg); a1*sind(theta01_deg); d1];
P14 = P04 - P01;
P14L = sqrt(P14(1)^2 + P14(2)^2 + P14(3)^2);

% l1 is the distance from joint 2 to joint 4
l1 = sqrt(a3^2 + d2^2);

% Cosine law to find angle phi
numerator_phi = l1^2 + a2^2 - P14L^2;
denominator_phi = 2 * l1 * a2;
phi = acosd(numerator_phi / denominator_phi);

% Angle zeta
zeta = atan2d(d2, a3);

% Theta 3 according to equation (2) in the paper
theta03_deg = phi - zeta - 180;
fprintf('Theta 3 = %.2f degrees\n', theta03_deg);