% ---------------------------------------------------------
%  OPTIMIZED INVERSE JACOBIAN SOLVER (Using Toolbox Math)
% ---------------------------------------------------------
clc; clear; close all;

% --- 1. ROBOT SETUP (Same Kuka Parameters) ---
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;  

L(1) = Link([0,      d1,     a1,        pi/2]); 
L(2) = Link([pi/2,   0,      a2,       -pi]);
L(3) = Link([0,      0,      a3,       -pi/2]);
L(4) = Link([0,      d2,     0,        pi/2]);
L(5) = Link([0,      0,      0,        -pi/2]);
L(6) = Link([0,      d3,     0,        0]);


Kuka = SerialLink(L, 'name', 'My_Kuka');

% --- 2. SIMULATION SETTINGS ---
q_current = deg2rad([0, 90, 0, 0, 0, 0]); 

% THE GOAL: Move Straight DOWN (-Z) at 50 mm/s
% [Vx; Vy; Vz; wx; wy; wz]
V_desired = [30; -30; -50; 1; 0.5; 0]; 

dt = 0.05;          
total_time = 10.0;   

% --- 3. SETUP PLOT ---
figure('Name', 'RTB Jacobian Control', 'Color', 'w');
Kuka.plot(q_current, 'workspace', [-400 400 -400 400 -1 400]);
hold on;

% Setup Trail Plotting
h_traj = plot3(0,0,0, 'b.', 'MarkerSize', 5); % Blue dots
traj_hist = [];

disp('--- STARTING ANIMATION ---');

% --- 4. THE OPTIMIZED LOOP ---
for t = 0:dt:total_time
    
    % ----------------------------------------------------
    % STEP A: CALCULATE JACOBIAN (The Peter Corke Way)
    % ----------------------------------------------------
    % jacob0 calculates the Jacobian in the WORLD FRAME (Base frame)
    % This replaces your entire 20-line numerical loop.
    J = Kuka.jacob0(q_current);
    
    % ----------------------------------------------------
    % STEP B: INVERSE KINEMATICS (Velocity Level)
    % ----------------------------------------------------
    % Using Pseudo-Inverse (pinv) to handle singularities gracefully
    q_dot = pinv(J) * V_desired;
    
    % ----------------------------------------------------
    % STEP C: INTEGRATION (Move Robot)
    % ----------------------------------------------------
    q_current = q_current + (q_dot' * dt);
    
    % ----------------------------------------------------
    % STEP D: VISUALIZATION
    % ----------------------------------------------------
    Kuka.animate(q_current);
    
    % Update Trail (Extract position from Forward Kinematics)
    T_now = double(Kuka.fkine(q_current));
    pos = T_now(1:3,4);
    
    traj_hist = [traj_hist, pos];
    set(h_traj, 'XData', traj_hist(1,:), 'YData', traj_hist(2,:), 'ZData', traj_hist(3,:));
    
    drawnow;
end

disp('--- FINISHED ---');