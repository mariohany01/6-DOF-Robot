%this is same code Moving_pose_with_jacobian but by gemini variables
clc; clear; close all;

%% 1. Robot Definition (DH Parameters)
% Parameters
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;  

% Define Links: Link([theta, d, a, alpha])
L(1) = Link([0      d1     a1        pi/2]); 
L(2) = Link([pi/2   0      a2       -pi]);
L(3) = Link([0      0      a3       -pi/2]);
L(4) = Link([0      d2     0        pi/2]);
L(5) = Link([0      0      0        -pi/2]);
L(6) = Link([0      d3     0        0]);

Kuka = SerialLink(L, 'name', 'My_Kuka');

%% 2. Initial State Calculation
q_home = [0 pi/2 0 0 0 0]; % Current Joint Configuration

% Visualize
Kuka.plot(q_home, 'workspace', [-400 400 -400 400 -1 400]);

% Calculate Forward Kinematics at Home Position (T at q)
T_current = Kuka.fkine(q_home);
T_current_mat = double(T_current); % Convert SE3 object to 4x4 Matrix

disp("--- Initial State ---")
disp("Current Transformation Matrix (T):");
disp(T_current_mat);

%% 3. Perturbation (Finite Difference Step)
delta_q = 1e-6; % A tiny change in angle

% Add perturbation ONLY to Joint 1
q_perturbed = q_home + [delta_q, 0, 0, 0, 0, 0];

% Calculate Forward Kinematics at Perturbed Position (T at q + dq)
T_perturbed = Kuka.fkine(q_perturbed);
T_perturbed_mat = double(T_perturbed);

%% 4. Calculate Numerical Derivative of T
% Formula: (T(q+dq) - T(q)) / dq
dT_dq1 = (T_perturbed_mat - T_current_mat) / delta_q;

disp("--- Derivative ---")
disp("Derivative of T w.r.t Joint 1 (dT/dq1):");
disp(dT_dq1);

%% 5. Extract Linear Velocity Vector (v)
% The first 3 rows of the 4th column represent the change in position
linear_velocity = dT_dq1(1:3, 4);

disp("--- Jacobian Components ---")
disp("Linear Velocity (Top half of Jacobian Col 1):");
disp(linear_velocity);

%% 6. Extract Angular Velocity Vector (omega)
% Extract the Rotation Matrix R from the current T
R_current = T_current_mat(1:3, 1:3);

% Extract the Derivative of Rotation Matrix (dR/dq)
dR_dq1 = dT_dq1(1:3, 1:3);

% Calculate Skew-Symmetric Matrix S = dR * inv(R)
% Logic: dR/dt = S(omega) * R  ->  S(omega) = dR * inv(R)
Skew_Symmetric_Mat = dR_dq1 * inv(R_current);

% Use 'vex' operator to convert 3x3 Skew matrix to 3x1 Vector
angular_velocity = vex(Skew_Symmetric_Mat); % Previously "Lesgo"

disp("Angular Velocity (Bottom half of Jacobian Col 1):");
disp(angular_velocity);

%% 7. Final Jacobian Column
% Combine Linear and Angular velocity into a 6x1 vector
Jacobian_Col_1 = [linear_velocity; angular_velocity];

disp("--- Final Result ---")
disp("Jacobian Column 1 (Geometric):");
disp(Jacobian_Col_1);

%% 8. Verification (Optional)
% Compare with the toolbox's built-in function
disp("--- Verification ---")
J_toolbox = Kuka.jacob0(q_home);
disp("Toolbox Calculated Column 1:");
disp(J_toolbox(:,1));