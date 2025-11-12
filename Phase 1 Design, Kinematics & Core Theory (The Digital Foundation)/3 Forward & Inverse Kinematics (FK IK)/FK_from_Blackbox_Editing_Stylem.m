% Clear workspace, command window, and close all figures
clc;
close all;
clear;

% Define Denavit-Hartenberg (DH) parameters for the robot links
% Units: lengths in mm, angles in radians (converted from degrees if needed)
a1 = 37.510;  % Link 1 length
a2 = 138.10;  % Link 2 length
a3 = 28.20;   % Link 3 length
d1 = 135.80;  % Link 1 offset
d2 = 160.00;  % Link 4 offset
d3 = 15.00;   % Link 6 offset

% Define joint angles in degrees (user inputs)
theta_01_deg = 0;     % Joint 1 angle in degrees
theta_02_deg = 90;    % Joint 2 angle in degrees
theta_03_deg = 0;     % Joint 3 angle in degrees
theta_04_deg = 0;     % Joint 4 angle in degrees
theta_05_deg = 0;     % Joint 5 angle in degrees
theta_06_deg = 0;     % Joint 6 angle in degrees

% Convert joint angles from degrees to radians for use in the robot model
q = [theta_01_deg, theta_02_deg, theta_03_deg, theta_04_deg, theta_05_deg, theta_06_deg] * (pi / 180);

% Define the robot links using DH parameters (now standard DH)
% Link([theta, d, a, alpha], 'standard') - standard DH convention
L(1) = Link([0, d1, a1, pi/2], 'standard');      % Joint 1: Revolute joint
L(2) = Link([pi/2, 0, a2, -pi], 'standard');    % Joint 2: Revolute joint
L(3) = Link([0, 0, a3, -pi/2], 'standard');     % Joint 3: Revolute joint
L(4) = Link([0, d2, 0, pi/2], 'standard');      % Joint 4: Revolute joint
L(5) = Link([0, 0, 0, -pi/2], 'standard');      % Joint 5: Revolute joint
L(6) = Link([0, d3, 0, 0], 'standard');         % Joint 6: Revolute joint

% Create the serial-link robot model
Rob = SerialLink(L, 'name', 'RRRRRR');  % 6-DOF robot with all revolute joints

% Plot the robot in the specified configuration with workspace limits
try
    Rob.plot(q, 'workspace', [-400, 400, -400, 400, -1, 500]);
    disp('Robot plot generated successfully.');
catch ME
    disp(['Error plotting robot: ', ME.message]);
end

% Compute forward kinematics for the given joint configuration
T = Rob.fkine(q);

% Convert the SE3 object to a numeric 4x4 transformation matrix for manipulation
T_matrix = double(T);

% Clean the transformation matrix by setting very small values (below threshold) to zero
% This improves readability by eliminating numerical noise from floating-point precision
threshold = 1e-10;  % Adjust threshold as needed for your precision requirements
T_clean = T_matrix;
T_clean(abs(T_clean) < threshold) = 0;

% Display the cleaned forward kinematics transformation matrix
disp('Forward Kinematics Transformation Matrix (T) - Small values set to zero:');
disp(T_clean);

% Manual forward kinematics computation for verification using provided equations
% Extract joint angles from q (already in radians)
theta1 = q(1); theta2 = q(2); theta3 = q(3); theta4 = q(4); theta5 = q(5); theta6 = q(6);

% Define trigonometric abbreviations for simplicity (corrected)
C1 = cos(theta1); S1 = sin(theta1);
C2 = cos(theta2); S2 = sin(theta2);
C3 = cos(theta3); S3 = sin(theta3);
C4 = cos(theta4); S4 = sin(theta4);
C5 = cos(theta5); S5 = sin(theta5);
C6 = cos(theta6); S6 = sin(theta6);
C23 = cos(theta2)*cos(theta3) + sin(theta2)*sin(theta3);  % cos(theta2 - theta3)
S23 = sin(theta2)*cos(theta3) - cos(theta2)*sin(theta3);  % sin(theta2 - theta3)

% Compute rotation matrix elements (first column: n_xf, n_yf, n_zf)
n_xf = C6 * (C5 * (S1 * S4 - S23 * C1 * C4) + C23 * C1 * S5) + S6 * (C4 * S1 + S23 * C1 * S4);
n_yf = -C6 * (C5 * (C1 * S4 + S23 * C4 * S1) - C23 * S1 * S5) - S6 * (C1 * C4 - S23 * S1 * S4);
n_zf = C6 * (S23 * S5 + C23 * C4 * C5) - C23 * S4 * S6;

% Compute rotation matrix elements (second column: o_xf, o_yf, o_zf)
o_xf = C6 * (C4 * S1 + S23 * C1 * S4) - S6 * (C5 * (S1 * S4 - S23 * C1 * C4) + C23 * C1 * S5);
o_yf = S6 * (C5 * (C1 * S4 + S23 * C4 * S1) - C23 * S1 * S5) - C6 * (C1 * C4 - S23 * S1 * S4);
o_zf = -S6 * (S23 * S5 + C23 * C4 * C5) - C23 * C6 * S4;

% Compute rotation matrix elements (third column: a_xf, a_yf, a_zf)
a_xf = C23 * C1 * C5 - S5 * (S1 * S4 - S23 * C1 * C4);
a_yf = S5 * (C1 * S4 + S23 * C4 * S1) + C23 * C5 * S1;
a_zf = S23 * C5 - C23 * C4 * S5;

% Compute position vector elements (p_xf, p_yf, p_zf)
p_xf = d2 * C23 * C1 - C1 * (a2 * S2 - a1 + a3 * S23) - d3 * (S5 * (S1 * S4 - S23 * C1 * C4) - C23 * C1 * C5);
p_yf = d3 * (S5 * (C1 * S4 + S23 * C4 * S1) + C23 * C5 * S1) - S1 * (a2 * S2 - a1 + a3 * S23) + d2 * C23 * S1;
p_zf = d1 + a2 * C2 + d3 * (S23 * C5 - C23 * C4 * S5) + a3 * C23 + d2 * S23;

% Assign to manual variables for comparison
PX_manual = p_xf;
PY_manual = p_yf;
PZ_manual = p_zf;

% Construct the manual rotation matrix
R_manual = [n_xf, o_xf, a_xf;
            n_yf, o_yf, a_yf;
            n_zf, o_zf, a_zf];

% Compute roll-pitch-yaw from the manual rotation matrix
rpy_manual = tr2rpy(R_manual);  % Returns [roll, pitch, yaw] in radians

% Extract toolbox values for comparison
pos_toolbox = transl(T_matrix);
rpy_toolbox = tr2rpy(T_matrix);

% Display comparison using fprintf for robust formatting (avoids concatenation issues)
disp('Position Comparison (Toolbox vs Manual):');
fprintf('Toolbox: %.6f %.6f %.6f\n', pos_toolbox(1), pos_toolbox(2), pos_toolbox(3));
fprintf('Manual:  %.6f %.6f %.6f\n', PX_manual, PY_manual, PZ_manual);
disp('Orientation (RPY) Comparison (Toolbox vs Manual):');
fprintf('Toolbox: %.6f %.6f %.6f\n', rpy_toolbox(1), rpy_toolbox(2), rpy_toolbox(3));
fprintf('Manual:  %.6f %.6f %.6f\n', rpy_manual(1), rpy_manual(2), rpy_manual(3));

% Optional: Uncomment the line below to open the interactive teaching interface
% Rob.teach;