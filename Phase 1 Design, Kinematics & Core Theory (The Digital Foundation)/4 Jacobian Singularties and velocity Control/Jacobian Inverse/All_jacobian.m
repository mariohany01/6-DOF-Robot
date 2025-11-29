% ---------------------------------------------------------
%  INVERSE JACOBIAN SOLVER WITH ANIMATION
%  Goal: Move End-Effector in a STRAIGHT LINE using J_inverse
% ---------------------------------------------------------

clc; clear; close all;

% --- 1. ROBOT SETUP (Your Kuka Parameters) ---
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
% Start Pose
q_current = deg2rad([0, 90, 0, 0, 0, 0]); 

% THE GOAL: Move Straight DOWN (-Z) at 50 mm/s
V_desired = [0; 0; -40; 0; 0; 0]; 

% Simulation Time
dt = 0.05;          % Update every 0.05 seconds
total_time = 3.0;   % Run for 3 seconds
steps = total_time / dt;

% Math Delta for Jacobian Formula
delta_math = 1e-6; 

% --- 3. SETUP PLOT ---
figure('Name', 'Inverse Jacobian Animation', 'Color', 'w');
Kuka.plot(q_current, 'workspace', [-400 400 -400 400 -1 400]);
hold on;
h_traj = plot3(0,0,0, 'r.', 'MarkerSize', 5); % Red trail
traj_hist = [];

disp('--- STARTING ANIMATION ---');
disp(['Target Linear  Velocity: [', num2str(V_desired(1:3)'), '] mm/s']);
disp(['Target Angular Velocity: [', num2str(V_desired(4:6)'), '] rad/s']);


% --- 4. THE ANIMATION LOOP ---
for t = 0:dt:total_time
    
    % ====================================================
    % STEP A: CALCULATE JACOBIAN (Numerical Method)
    % ====================================================
    J_numerical = zeros(6, 6); 
    
    % Current Pose
    T0 = double(Kuka.fkine(q_current));
    R0 = T0(1:3,1:3);
    P0 = T0(1:3,4);
    
    for i = 1:6
        % Perturb Joint i
        q_pert = q_current;
        q_pert(i) = q_pert(i) + delta_math;
        
        % Perturbed Pose
        T1 = double(Kuka.fkine(q_pert));
        R1 = T1(1:3,1:3);
        P1 = T1(1:3,4);
        
        % Fill Columns
        J_numerical(1:3, i) = (P1 - P0) / delta_math;       % Linear
        
        dR = (R1 - R0) / delta_math;
        J_numerical(4:6, i) = Vex(dR * R0');                % Angular
    end
    
    % ====================================================
    % STEP B: INVERSE KINEMATICS (Velocity Level)
    % ====================================================
    % q_dot = J_inverse * V_desired
    q_dot = pinv(J_numerical) * V_desired;
    
    % ====================================================
    % STEP C: MOVE ROBOT (Integration)
    % ====================================================
    % q_new = q_old + (speed * time)
    % angle_new = angle_old + ( angular velocity * time)
    %                                              0.05
    %You are taking the instantaneous velocity (q_dot), pretending it stays constant 
    % for a tiny fraction of a second (dt), and "teleporting" the robot to that new spot.
    q_current = q_current + (q_dot' * dt);
    
    % ====================================================
    % STEP D: UPDATE PLOT
    % ====================================================
    Kuka.animate(q_current);
    
    % Update Trail
    T_now = double(Kuka.fkine(q_current));
    pos = T_now(1:3,4);
    %Trajectory History is adding the new trajectory to a bucket always 
    traj_hist = [traj_hist, pos];
    %We use set to tell that specific line: "Hey, update your X, Y, and Z 
    %data to include this new list of points." This is incredibly fast and efficient
    set(h_traj, 'XData', traj_hist(1,:), 'YData', traj_hist(2,:), 'ZData', traj_hist(3,:));
    
    drawnow;
end

disp('--- FINISHED ---');


% --- HELPER FUNCTION ---
function v = Vex(S)
    wx = 0.5 * (S(3,2) - S(2,3));
    wy = 0.5 * (S(1,3) - S(3,1));
    wz = 0.5 * (S(2,1) - S(1,2));
    v = [wx; wy; wz];
end