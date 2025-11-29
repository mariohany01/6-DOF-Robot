% ---------------------------------------------------------
%  JACOBIAN STEP-BY-STEP (CUSTOM KUKA ROBOT)
%  Goal: Move Kuka Joint 1 from 0 to pi/4
%  Demonstrates: Numerical Jacobian on a 6-DOF SerialLink
% ---------------------------------------------------------

clc; clear; close all;

% --- 1. ROBOT SETUP (Your Parameters) ---
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;  

% Define Links using Robotics Toolbox (Standard DH)
% Syntax: Link([theta, d, a, alpha])
% Note: The 'sigma' (joint type) defaults to 0 (revolute)
L(1) = Link([0,      d1,     a1,        pi/2]); 
L(2) = Link([pi/2,   0,      a2,       -pi]);   % Includes pi/2 offset
L(3) = Link([0,      0,      a3,       -pi/2]);
L(4) = Link([0,      d2,     0,        pi/2]);
L(5) = Link([0,      0,      0,        -pi/2]);
L(6) = Link([0,      d3,     0,        0]);

Kuka = SerialLink(L, 'name', 'My_Kuka');

% --- 2. SIMULATION CONFIGURATION ---
% The "Measuring Stick" (For the Math Formula)
delta_math = 1e-6; 

% The "Walking Speed" (Movement)
target_angle = pi/2;        % Target for Joint 1 (approx 0.785 rad)
speed_q1     = 0.5;         % Speed: 0.5 rad/sec
dt           = 0.1;         % Time step (Update rate)

% Initial State (6 Joints)
q_current = [0, pi/2, 0, 0, 0, 0]; 

% Setup Plot
figure('Name', 'Kuka Jacobian Simulation', 'Color', 'w');
Kuka.plot(q_current ,'workspace', [-400 400 -400 400 -1 400]); % Draw the robot once to initialize
hold on;
h_traj = plot3(0, 0, 0, 'r.', 'MarkerSize', 5); % Red dots for path
traj_hist = []; 

disp('--- STARTING KUKA SIMULATION ---');
disp(['Target q1: ', num2str(target_angle), ' rad']);
disp('--------------------------------');

% --- 3. THE MAIN LOOP ---
time = 0;
step_count = 1;

while q_current(1) < target_angle
    
    % ============================================================
    % PART A: THE "CHEAT SHEET" CALCULATIONS (Virtual)
    % ============================================================
    
    % 1. Calculate T0 (Current Pose)
    % We use double() to ensure we get a 4x4 Matrix, not an SE3 object
    T0 = double(Kuka.fkine(q_current));
    
    % 2. Create Perturbed Q (Add delta ONLY to Joint 1)
    q_perturbed = q_current;
    q_perturbed(1) = q_perturbed(1) + delta_math;
    
    % 3. Calculate T1 (Perturbed Pose)
    T1 = double(Kuka.fkine(q_perturbed));
    
    % 4. APPLY YOUR FORMULAS
    % ------------------------------------------------
    % Upper Part (Linear Velocity)
    J_upper = (T1(1:3,4) - T0(1:3,4)) / delta_math;
    
    % Lower Part (Angular Velocity)
    R0 = T0(1:3,1:3);
    R1 = T1(1:3,1:3);
    
    % Your specific Vex formula:
    Matrix_For_Vex = ((R1 - R0) / delta_math) * inv(R0); % or R0'
    J_lower = Vex(Matrix_For_Vex);
    
    % Combine to get Column 1 of Jacobian
    J_Col1 = [J_upper; J_lower];
    % ------------------------------------------------
    
    % Print to Command Window
    fprintf('Step %d: q1=%.3f | J1_Linear_X=%.2f | J1_Linear_Y=%.2f\n', ...
        step_count, q_current(1), J_Col1(1), J_Col1(2));
    
    
    % ============================================================
    % PART B: THE "REAL" MOVE (Physical)
    % ============================================================
    
    % Calculate the "Walking Step" (Distance = Speed * Time)
    move_step = speed_q1 * dt;
    
    % Update Position
    q_current(1) = q_current(1) + move_step;
    
    % Cap at target
    if q_current(1) > target_angle
        q_current(1) = target_angle;
    end
    
    % ============================================================
    % PART C: PLOTTING
    % ============================================================
    
    % Update Robot Visual
    Kuka.animate(q_current); 
    
    % Track End Effector Trace
    T_plot = double(Kuka.fkine(q_current));
    pos = T_plot(1:3,4);
    traj_hist = [traj_hist, pos]; 
    
    % Update red dot trail
    set(h_traj, 'XData', traj_hist(1,:), ...
                'YData', traj_hist(2,:), ...
                'ZData', traj_hist(3,:));
    
    title(['Kuka Moving: q1 = ', num2str(rad2deg(q_current(1)), '%.1f'), ' deg']);
    drawnow;
    
    time = time + dt;
    step_count = step_count + 1;
end

disp('--- TARGET REACHED ---');

% ---------------------------------------------------------
% HELPER FUNCTION
% ---------------------------------------------------------
% The VEX Operator: Extracts [wx; wy; wz] from Skew-Symmetric Matrix
function v = Vex(S)
    wx = 0.5 * (S(3,2) - S(2,3));
    wy = 0.5 * (S(1,3) - S(3,1));
    wz = 0.5 * (S(2,1) - S(1,2));
    v = [wx; wy; wz];
end