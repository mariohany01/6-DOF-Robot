% ==============================================================================
% 6-DOF ROBOT INVERSE KINEMATICS SOLVER (GEOMETRIC DECOMPOSITION)
% ==============================================================================
%
% DESCRIPTION:
%   This script implements an analytical Inverse Kinematics (IK) solution for a 
%   6-DOF serial manipulator featuring mechanical offsets and a spherical wrist 
%   (KUKA-style architecture). The solution employs a "Geometric Decoupling" 
%   strategy, splitting the problem into two subsystems to ensure computational 
%   efficiency and singularity robustness.
%
% METHODOLOGY:
%   The solver decomposes the 6-DOF problem into:
%   1. POSITIONING PROBLEM (Joints 1-3): 
%      - The Wrist Center (P04) is isolated by backtracking from the End-Effector 
%        coordinate (P06) along the approach vector (Z-axis) by distance d3.
%      - Theta 1 is derived via projection onto the XY plane.
%      - Theta 2 and Theta 3 are solved using a planar geometric approach 
%        (Law of Cosines) projected onto the plane defined by Theta 1.
%        Crucially, this handles the fixed shoulder/elbow offsets (zeta).
%
%   2. ORIENTATION PROBLEM (Joints 4-6):
%      - With Theta 1-3 known, the rotation matrix R03 is computed.
%      - The Spherical Wrist orientation (R46) is isolated: R46 = inv(R03) * R06.
%      - Theta 4, 5, and 6 are extracted from R46 using Z-Y-Z Euler angle 
%        conversion logic.
%
% SYSTEM PARAMETERS (DH Convention):
%   The script uses specific link lengths (a_i) and offsets (d_i) defined in 
%   the setup section. These must match the physical CAD dimensions.
%
% INPUT:
%   - Num_T_final: A 4x4 Homogeneous Transformation Matrix representing the 
%     Desired End-Effector Pose (Rotation + Position).
%
% OUTPUT:
%   - Theta01 - Theta06: The six joint angles (in degrees) required to achieve
%     the target pose.
%
% LIMITATIONS & CONSTRAINTS:
%   - Solution assumes an "Elbow-Up" configuration (Theta 3 >= 0).
%   - Singularity handling is managed via atan2d, but standard wrist 
%     singularities (Theta 5 = 0) may require additional handling in trajectory 
%     loops.
%
% ==============================================================================

% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures
fprintf('Running 6-DOF Robot Inverse Kinematic Analysis...\n\n');

% Robot Parameters (DH Convention)
a1 = 37.5;
a2 = 160;
a3 = 15;
d1 = 135.8;
d2 = 138.1;
d3 = 28.2;

% Desired End-Effector Pose (4x4 Homogeneous Transformation Matrix)
% Example: Robot zero pose (0, 90, 0, 0, 0, 0)
%Num_T_final = [
%    0         0         1.0000  203.8000;
%    0         -1.0000   0        0;
%    1.0000    0         0       310.8000;
%    0         0         0       1.0000
%];
Num_T_final = [
    0.7202   -0.0200    0.6935 -244.3321;
    0.6928    0.0741   -0.7174  -15.6234;
   -0.0370    0.9971    0.0672  -37.1346;
         0         0         0    1.0000;
];
% --- INVERSE KINEMATICS CALCULATIONS ---
fprintf('==================================================\n');
fprintf('Inverse Kinematics:\n');
fprintf('==================================================\n');

% Extract position and compute wrist center
P06 = Num_T_final(1:3, 4);
P46 = d3 * Num_T_final(1:3, 3);
P04 = P06 - P46;

% Theta 1
theta01 = atan2d(P04(2), P04(1));
fprintf('Theta 01 ......done \n');

% Constants for Theta 2 and Theta 3
p = sqrt(P04(2)^2 + P04(1)^2);
xp = p - a1;
zp = P04(3) - d1;
r = sqrt(xp^2 + zp^2);
l = sqrt(a3^2 + d2^2);
zeta = atan2d(d2, a3);
c = (a2^2 + l^2 - r^2) / (2 * a2 * l);
gamma = acosd(c);
theta03 = 180 - (gamma + zeta);
fprintf('Theta 03 ......done \n');

% Theta 2
beta01 = atan2d(zp, xp);
beta02_numerator = a2^2 + r^2 - l^2;
beta02_denominator = 2 * a2 * r;
cos_beta02_ratio = beta02_numerator / beta02_denominator;
beta02 = acosd(cos_beta02_ratio);
theta02 = beta01 + beta02;
fprintf('Theta 02 ......done \n');

% Compute transformation matrices for orientation
theta01_rad = deg2rad(theta01);
theta02_rad = deg2rad(theta02);
theta03_rad = deg2rad(theta03);

A1 = [
    cos(theta01_rad) 0              sin(theta01_rad) a1*cos(theta01_rad);
    sin(theta01_rad) 0             -cos(theta01_rad) a1*sin(theta01_rad);
    0            1              0             d1;
    0            0              0             1
];
A2 = [
    cos(theta02_rad)  sin(theta02_rad)  0             a2*cos(theta02_rad);
    sin(theta02_rad) -cos(theta02_rad)  0             a2*sin(theta02_rad);
    0             0            -1             0;
    0             0             0             1
];
A3 = [
    cos(theta03_rad)  0             -sin(theta03_rad) a3*cos(theta03_rad);
    sin(theta03_rad)  0              cos(theta03_rad) a3*sin(theta03_rad);
    0            -1              0             0;
    0             0              0             1
];

A03 = A1 * A2 * A3;  % T_0_3

% Theta 5
Z03 = A03(1:3, 3);
Z06 = Num_T_final(1:3, 3);
dot_product = dot(Z03, Z06);
mag1 = norm(Z03);
mag2 = norm(Z06);
cos_theta = dot_product / (mag1 * mag2);
theta05 = acosd(cos_theta);
fprintf('Theta 05 ......done \n');

% Compute A46
A03_inv = inv(A03);
A46 = A03_inv * Num_T_final;

% Theta 4
dummy = atan2d(A46(2,4), A46(1,4));
theta04 = dummy + 180;
fprintf('Theta 04 ......done \n');

% Theta 6
theta06 = -atan2d(A46(3,2), A46(3,1));
fprintf('Theta 06 ......done \n');

fprintf('==================================================\n');

% --- OUTPUT RESULTS ---
fprintf('Theta Inverse Kinematics values\n');
fprintf('Theta 01 = %f\n', theta01);
fprintf('Theta 02 = %f\n', theta02);
fprintf('Theta 03 = %f\n', theta03);
fprintf('Theta 04 = %f\n', theta04);
fprintf('Theta 05 = %f\n', theta05);
fprintf('Theta 06 = %f\n', theta06);