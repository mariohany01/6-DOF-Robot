% =========================================================
%  INVERSE DYNAMICS SIMULATION
%  Task: Pick from ground (LEFT), place on ground (RIGHT)
%  Cycle time: 4 seconds
%  Output: Required torque at each joint over time
% =========================================================
clc; clear; close all;

% =========================================================
%  SECTION 1 — TRAJECTORY DEFINITION
%  All positions in meters, in base frame
% =========================================================

% --- Key Poses (joint angles in radians) ---
% Home: upright starting position (facing front)
q_home  = deg2rad([0,   90,  0,  0,  0, 0]);

% Approach ground (LEFT side, pitched down)
% J1 = 90 deg rotates the base to the left
q_approach_left = deg2rad([90,   45, 45,  0,  0, 0]);

% Grasp position (LEFT ground, shoulder down, elbow bent)
q_grasp_left    = deg2rad([90,   25, 60,  0, -10, 0]);

% Lift off ground (LEFT side)
q_lift_left     = deg2rad([90,   65, 20,  0, 20, 0]);

% Rotate to RIGHT side (180 deg base swing from 90 to -90)
q_swing_right   = deg2rad([-90, 65, 20,  0, 20, 0]);

% Place position (RIGHT ground)
q_place_right   = deg2rad([-90, 25, 60,  0, -10, 0]);

% Retract after place (RIGHT side)
q_retract_right = deg2rad([-90, 45, 45,  0,  0, 0]);

% Return home (facing front)
q_final         = deg2rad([0,   90,  0,  0,  0, 0]);

% --- Waypoints and timing ---
% Each row: [waypoint, duration to reach it in seconds]
waypoints = {q_home,             0.0;   % start
             q_approach_left,    0.6;   % move left & down
             q_grasp_left,       0.4;   % descend to grasp
             q_lift_left,        0.4;   % lift object
             q_swing_right,      1.0;   % swing base left to right
             q_place_right,      0.4;   % descend to place
             q_retract_right,    0.3;   % retract
             q_final,            0.9};  % return home
% Total = 4.0 seconds

% =========================================================
%  SECTION 2 — TRAJECTORY INTERPOLATION
%  Cubic spline through waypoints — smooth q, qdot, qddot
% =========================================================
dt   = 0.01;   % 10ms timestep
t_total = 4.0;
t_sim   = 0:dt:t_total;
N       = length(t_sim);

% Build time vector for waypoints
t_wp = zeros(size(waypoints,1), 1);
for i = 2:size(waypoints,1)
    t_wp(i) = t_wp(i-1) + waypoints{i,2};
end

% Build joint angle matrix [n_waypoints x 6]
Q_wp = zeros(size(waypoints,1), 6);
for i = 1:size(waypoints,1)
    Q_wp(i,:) = waypoints{i,1};
end

% Interpolate each joint with cubic spline
Q    = zeros(N, 6);   % positions
Qd   = zeros(N, 6);   % velocities
Qdd  = zeros(N, 6);   % accelerations

for j = 1:6
    % Cubic spline interpolation
    pp      = spline(t_wp, Q_wp(:,j));
    Q(:,j)  = ppval(pp, t_sim);

    % First derivative (velocity)
    pp_d    = fnder(pp, 1);
    Qd(:,j) = ppval(pp_d, t_sim);

    % Second derivative (acceleration)
    pp_dd    = fnder(pp, 2);
    Qdd(:,j) = ppval(pp_dd, t_sim);
end

% =========================================================
%  SECTION 3 — INVERSE DYNAMICS
%  tau = M(q)*qddot + C(q,qdot)*qdot + G(q)
% =========================================================
fprintf('Running inverse dynamics simulation...\n');

TAU = zeros(N, 6);   % torque history

for k = 1:N
    q_k   = Q(k,:);
    qd_k  = Qd(k,:);
    qdd_k = Qdd(k,:);

    % Mass matrix
    M_k = get_M(q_k(1),q_k(2),q_k(3), ...
                q_k(4),q_k(5),q_k(6));

    % Coriolis
    C_k = get_C(q_k, qd_k);

    % Gravity
    G_k = get_G(q_k(1),q_k(2),q_k(3), ...
                q_k(4),q_k(5),q_k(6));

    % Equation of motion
    TAU(k,:) = (M_k * qdd_k' + C_k * qd_k' + G_k)';
end

fprintf('Done.\n');

% =========================================================
%  SECTION 4 — TORQUE RESULTS & MOTOR SIZING DATA
% =========================================================
fprintf('\n========================================\n');
fprintf('  JOINT TORQUE REQUIREMENTS\n');
fprintf('========================================\n');

joint_names = {'J1 Base',  'J2 Shoulder', 'J3 Elbow', ...
               'J4 Wrist1','J5 Wrist2',   'J6 Wrist3'};

peak_tau     = zeros(1,6);
cont_tau_rms = zeros(1,6);

for j = 1:6
    peak_tau(j)     = max(abs(TAU(:,j)));
    cont_tau_rms(j) = rms(TAU(:,j));
    fprintf('%-14s  Peak: %7.4f N·m   RMS: %7.4f N·m\n', ...
            joint_names{j}, peak_tau(j), cont_tau_rms(j));
end

fprintf('\nMotor sizing rule:\n');
fprintf('  Continuous motor torque >= RMS torque\n');
fprintf('  Peak motor torque       >= Peak torque x 1.5 (safety factor)\n');

% =========================================================
%  SECTION 5 — PLOTS
% =========================================================
figure('Name','Trajectory','Color','w','Position',[100 100 1200 800]);

% --- Plot 1: Joint angles ---
subplot(3,2,1);
plot(t_sim, rad2deg(Q), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Joint Angles');
legend(joint_names, 'Location','best','FontSize',8);
grid on;

% --- Plot 2: Joint velocities ---
subplot(3,2,2);
plot(t_sim, rad2deg(Qd), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (deg/s)');
title('Joint Velocities');
legend(joint_names, 'Location','best','FontSize',8);
grid on;

% --- Plot 3: Joint accelerations ---
subplot(3,2,3);
plot(t_sim, rad2deg(Qdd), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Accel (deg/s²)');
title('Joint Accelerations');
legend(joint_names, 'Location','best','FontSize',8);
grid on;

% --- Plot 4: Joint torques ---
subplot(3,2,4);
plot(t_sim, TAU, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Torque (N·m)');
title('Required Joint Torques');
legend(joint_names, 'Location','best','FontSize',8);
grid on;

% --- Plot 5: Peak torque bar chart ---
subplot(3,2,5);
bar(peak_tau, 'FaceColor', [0.2 0.6 0.9]);
set(gca, 'XTickLabel', joint_names, 'XTickLabelRotation', 30);
ylabel('Peak Torque (N·m)');
title('Peak Torque per Joint');
grid on;

% --- Plot 6: RMS torque bar chart ---
subplot(3,2,6);
bar(cont_tau_rms, 'FaceColor', [0.9 0.5 0.2]);
set(gca, 'XTickLabel', joint_names, 'XTickLabelRotation', 30);
ylabel('RMS Torque (N·m)');
title('Continuous (RMS) Torque per Joint');
grid on;

sgtitle('Left-to-Right Pick-and-Place Inverse Dynamics', 'FontSize', 14);

% =========================================================
%  SECTION 6 — ROBOT ANIMATION
% =========================================================
figure('Name','Robot Animation','Color','w');

% Build robot (DH in meters now)
a1=0.0375; a2=0.1600; a3=0.0150;
d1=0.1358; d2=0.1381; d3=0.0282;
L(1) = Link([0,    d1, a1,  pi/2]);
L(2) = Link([pi/2,  0, a2,  -pi]);
L(3) = Link([0,     0, a3, -pi/2]);
L(4) = Link([0,    d2,  0,  pi/2]);
L(5) = Link([0,     0,  0, -pi/2]);
L(6) = Link([0,    d3,  0,  0]);
Kuka = SerialLink(L, 'name', 'Kuka');

Kuka.plot(Q(1,:), 'workspace', [-0.6 0.6 -0.6 0.6 -0.1 0.7]);
hold on;

% Draw pickup zone (LEFT ground / Y-axis positive)
left_x = [-0.10 -0.10  0.10  0.10 -0.10];
left_y = [ 0.15  0.35  0.35  0.15  0.15];
ground_z = [ 0.00  0.00  0.00  0.00  0.00]; 
fill3(left_x, left_y, ground_z, [0.6 0.4 0.2], 'FaceAlpha', 0.5);
text(0, 0.25, 0.02, 'PICK', 'FontSize', 8, 'HorizontalAlignment','center');

% Draw drop zone (RIGHT ground / Y-axis negative)
right_x = [-0.10 -0.10  0.10  0.10 -0.10];
right_y = [-0.15 -0.35 -0.35 -0.15 -0.15];
fill3(right_x, right_y, ground_z, [0.2 0.6 0.2], 'FaceAlpha', 0.5);
text(0, -0.25, 0.02, 'DROP', 'FontSize', 8, 'HorizontalAlignment','center');

h_traj = plot3(nan, nan, nan, 'r-', 'LineWidth', 1.5);
traj_hist = [];

fprintf('\nPlaying animation...\n');
skip = 3;  % animate every 3rd frame for speed
for k = 1:skip:N
    Kuka.animate(Q(k,:));
    T_now = double(Kuka.fkine(Q(k,:)));
    traj_hist = [traj_hist, T_now(1:3,4)];
    set(h_traj, 'XData', traj_hist(1,:), ...
                'YData', traj_hist(2,:), ...
                'ZData', traj_hist(3,:));
    drawnow;
end

fprintf('\n========================================\n');
fprintf('  SIMULATION COMPLETE\n');
fprintf('  Use peak torques x 1.5 for motor selection\n');
fprintf('========================================\n');