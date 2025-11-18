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
R_current = T_current_mat(1:3, 1:3); 
disp("Calculating Jacobian using Finite Difference Method...");
disp("-----------------------------------------------------");

%% 3. Numerical Jacobian Calculation Loop
J_Numerical = zeros(6, 6); 
delta_q = 1e-6; 

for i = 1:6
    q_perturbed = q_home; 
    q_perturbed(i) = q_perturbed(i) + delta_q;
    
    T_new = Kuka.fkine(q_perturbed);
    T_new_mat = double(T_new);
    
    dT_dqi = (T_new_mat - T_current_mat) / delta_q;
    
    v_i = dT_dqi(1:3, 4);
    dR_dqi = dT_dqi(1:3, 1:3);
    S = dR_dqi * R_current'; 
    w_i = vex(S);
    
    J_Numerical(:, i) = [v_i; w_i];
end

%% 4. Display Results
disp("Final Numerical Jacobian Matrix (6x6):");
disp(J_Numerical);

%% 5. Verification
disp("--- Comparison with Toolbox ---");
J_Toolbox = Kuka.jacob0(q_home);
% Check if error is negligible
Error_Matrix = J_Numerical - J_Toolbox;
if max(abs(Error_Matrix(:))) < 1e-3
    disp("SUCCESS: The calculated Jacobian matches the Toolbox!");
else
    disp("WARNING: There is a significant difference.");
end

%% 6. ANIMATION: Rotate Around Theta 1 Only
disp("Starting Animation around Theta 1...");

% Define the range of motion: -180 to 180 degrees (Full Circle)
% linspace(start, end, number_of_frames)
theta1_range = linspace(-pi, pi, 100); 

% Define workspace to keep the camera steady [xmin xmax ymin ymax zmin zmax]
ws = [-500 500 -500 500 0 600];

for i = 1:length(theta1_range)
    % Copy the home configuration
    q_animate = q_home;
    
    % Update ONLY the first joint (Theta 1)
    q_animate(1) = theta1_range(i);
    
    % Plot the robot
    % 'fps' controls speed (frames per second)
    Kuka.plot(q_animate, 'workspace', ws, 'fps', 60); 
end

disp("Animation Complete.");