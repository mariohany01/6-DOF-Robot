clc; clear; close all;

%% 1. Robot Setup
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;  

L(1) = Link([0      d1     a1        pi/2]); 
L(2) = Link([pi/2   0      a2       -pi]);
L(3) = Link([0      0      a3       -pi/2]);
L(4) = Link([0      d2     0        pi/2]);
L(5) = Link([0      0      0        -pi/2]);
L(6) = Link([0      d3     0        0]);

Kuka = SerialLink(L, 'name', 'My_Kuka');

%% 2. Initial State
q_home = [0 pi/2 0 0 0 0];
T_current = Kuka.fkine(q_home);
T_current_mat = double(T_current);
R_current = T_current_mat(1:3, 1:3); % Extract Rotation Matrix once

disp("Calculating Jacobian using Finite Difference Method...");
disp("-----------------------------------------------------");

%% 3. Numerical Jacobian Calculation Loop

% Initialize an empty 6x6 matrix to store results
J_Numerical = zeros(6, 6); 

delta_q = 1e-6; % The perturbation value

for i = 1:6
    % --- A. Prepare the Perturbed Joint Vector ---
    % Start with the original angles
    q_perturbed = q_home; 
    
    % Add delta ONLY to the current joint 'i'
    q_perturbed(i) = q_perturbed(i) + delta_q;
    
    % --- B. Forward Kinematics for the Perturbed State ---
    T_new = Kuka.fkine(q_perturbed);
    T_new_mat = double(T_new);
    
    % --- C. Calculate Numerical Derivative (dT/dqi) ---
    dT_dqi = (T_new_mat - T_current_mat) / delta_q;
    
    % --- D. Linear Velocity (v) ---
    % Top 3 rows of the 4th column
    v_i = dT_dqi(1:3, 4);
    
    % --- E. Angular Velocity (omega) ---
    % Extract derivative of Rotation Matrix
    dR_dqi = dT_dqi(1:3, 1:3);
    
    % S(w) = dR * R_transpose (Note: R_transpose is same as inv(R))
    S = dR_dqi * R_current'; 
    
    % Extract vector from Skew-Symmetric matrix
    w_i = vex(S);
    
    % --- F. Assemble Column and Store ---
    J_col = [v_i; w_i];
    
    % Store this column in the i-th position of the big matrix
    J_Numerical(:, i) = J_col;
end

%% 4. Display Results
disp("Final Numerical Jacobian Matrix (6x6):");
disp(J_Numerical);

%% 5. Verification
disp("--- Comparison with Toolbox ---");
J_Toolbox = Kuka.jacob0(q_home);

disp("Final Tool box Jacobian Matrix (6x6):");
disp(J_Toolbox);

% Calculate the difference (Error)
Error_Matrix = J_Numerical - J_Toolbox;

% Check if error is negligible
if max(abs(Error_Matrix(:))) < 1e-3
    disp("SUCCESS: The calculated Jacobian matches the Toolbox!");
else
    disp("WARNING: There is a significant difference.");
end

disp("Difference Matrix:");
disp(Error_Matrix);