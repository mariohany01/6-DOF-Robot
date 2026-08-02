% =========================================================
%  INVERSE DYNAMICS — WORST CASE TRAJECTORY
%  Fully stretched LEFT → arc through ground → fully stretched RIGHT
%  Cycle time: 6 seconds
%  This is the hardest possible motion for torque sizing
% =========================================================
clc; clear; close all;

% =========================================================
%  SECTION 1 — ROBOT SETUP
% =========================================================
a1=0.0375; a2=0.1600; a3=0.0150;
d1=0.1358; d2=0.1381; d3=0.0282;
L(1) = Link([0,    d1, a1,  pi/2]);
L(2) = Link([pi/2,  0, a2,  -pi]);
L(3) = Link([0,     0, a3, -pi/2]);
L(4) = Link([0,    d2,  0,  pi/2]);
L(5) = Link([0,     0,  0, -pi/2]);
L(6) = Link([0,    d3,  0,  0]);
Kuka = SerialLink(L, 'name', 'Kuka');

% =========================================================
%  SECTION 2 — WAYPOINTS
%
%  MOTION DESCRIPTION:
%  1. Home (upright)
%  2. Fully stretched LEFT  — arm horizontal, pointing left
%  3. Arc DOWN through front — arm sweeps toward ground
%  4. Ground level FRONT    — arm fully stretched, pointing down-forward
%  5. Arc DOWN through back — continues arc under
%  6. Fully stretched RIGHT — arm horizontal, pointing right
%  7. Return home
%
%  "Fully stretched" = shoulder pitched out, elbow straight
%  J2 = 0° means shoulder horizontal
%  J3 = 0° means elbow fully extended (straight arm)
%  J1 rotates the base: +90=LEFT, 0=FRONT, -90=RIGHT
% =========================================================

% Home — upright
q_home = deg2rad([0, 90, 0, 0, 0, 0]);

% Fully stretched LEFT — base rotated 90° left, arm horizontal & straight
q_stretch_left = deg2rad([90, 0, 0, 0, 0, 0]);

% Arc point 1 — base at 45° left, arm angled 45° down
q_arc_left45   = deg2rad([45, -20, 10, 0, 20, 0]);

% Arc point 2 — base at 0° (front), arm pointing down-forward (ground reach)
% This is the lowest point — maximum gravity load on all joints
q_ground_front = deg2rad([0, -30, 20, 0, 30, 0]);

% Arc point 3 — base at -45° right, arm angled 45° down
q_arc_right45  = deg2rad([-45, -20, 10, 0, 20, 0]);

% Fully stretched RIGHT — base rotated 90° right, arm horizontal & straight
q_stretch_right = deg2rad([-90, 0, 0, 0, 0, 0]);

% Return home
q_final = deg2rad([0, 90, 0, 0, 0, 0]);

% =========================================================
%  SECTION 3 — TIMING
%  Total = 6 seconds
%  Slowest segments = the arc through ground (most stress)
% =========================================================
waypoints = {
    q_home,          0.0;    % start
    q_stretch_left,  1.0;    % move to full left stretch
    q_arc_left45,    0.8;    % begin arc downward
    q_ground_front,  1.0;    % reach ground — slowest, hardest
    q_arc_right45,   0.8;    % continue arc upward
    q_stretch_right, 1.0;    % reach full right stretch
    q_final,         1.4;    % return home (slow, controlled)
};
% Total = 6.0 seconds

% =========================================================
%  SECTION 4 — TRAJECTORY INTERPOLATION
% =========================================================
dt      = 0.01;
t_total = 6.0;
t_sim   = 0:dt:t_total;
N       = length(t_sim);

% Build waypoint time vector
t_wp = zeros(size(waypoints,1), 1);
for i = 2:size(waypoints,1)
    t_wp(i) = t_wp(i-1) + waypoints{i,2};
end

% Build joint angle matrix
Q_wp = zeros(size(waypoints,1), 6);
for i = 1:size(waypoints,1)
    Q_wp(i,:) = waypoints{i,1};
end

% Interpolate with cubic spline
Q   = zeros(N, 6);
Qd  = zeros(N, 6);
Qdd = zeros(N, 6);

for j = 1:6
    pp       = spline(t_wp, Q_wp(:,j));
    Q(:,j)   = ppval(pp, t_sim);
    pp_d     = fnder(pp, 1);
    Qd(:,j)  = ppval(pp_d, t_sim);
    pp_dd    = fnder(pp, 2);
    Qdd(:,j) = ppval(pp_dd, t_sim);
end

% =========================================================
%  SECTION 5 — INVERSE DYNAMICS
%  tau = M(q)*qddot + C(q,qdot)*qdot + G(q)
% =========================================================
fprintf('Running inverse dynamics — worst case trajectory...\n');

TAU = zeros(N, 6);

for k = 1:N
    q_k   = Q(k,:);
    qd_k  = Qd(k,:);
    qdd_k = Qdd(k,:);

    M_k = get_M(q_k(1),q_k(2),q_k(3),q_k(4),q_k(5),q_k(6));
    C_k = get_C(q_k, qd_k);
    G_k = get_G(q_k(1),q_k(2),q_k(3),q_k(4),q_k(5),q_k(6));

    TAU(k,:) = (M_k * qdd_k' + C_k * qd_k' + G_k)';
end

fprintf('Done.\n');

% =========================================================
%  SECTION 6 — RESULTS
% =========================================================
joint_names  = {'J1 Base','J2 Shoulder','J3 Elbow', ...
                'J4 Wrist1','J5 Wrist2','J6 Wrist3'};
peak_tau     = zeros(1,6);
cont_tau_rms = zeros(1,6);

fprintf('\n========================================\n');
fprintf('  WORST CASE TORQUE REQUIREMENTS\n');
fprintf('========================================\n');

for j = 1:6
    peak_tau(j)     = max(abs(TAU(:,j)));
    cont_tau_rms(j) = rms(TAU(:,j));
    fprintf('%-14s  Peak: %7.4f N·m   RMS: %7.4f N·m\n', ...
            joint_names{j}, peak_tau(j), cont_tau_rms(j));
end

% Show which time instant has max torque per joint
fprintf('\nPeak torque occurs at:\n');
for j = 1:6
    [~, idx] = max(abs(TAU(:,j)));
    fprintf('  %s: t = %.2f s\n', joint_names{j}, t_sim(idx));
end

fprintf('\nSafety factor 1.5× — motor must provide:\n');
for j = 1:6
    fprintf('  %-14s  %.4f N·m\n', joint_names{j}, peak_tau(j)*1.5);
end

% =========================================================
%  SECTION 7 — PLOTS
% =========================================================
figure('Name','Worst Case Trajectory','Color','w', ...
       'Position',[50 50 1300 850]);

subplot(3,2,1);
plot(t_sim, rad2deg(Q), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Joint Angles — Full Stretch Arc');
legend(joint_names,'Location','best','FontSize',8);
xline(t_wp(2),'--k'); xline(t_wp(4),'--r'); xline(t_wp(6),'--k');
text(t_wp(2), 5,'Left','FontSize',7,'HorizontalAlignment','center');
text(t_wp(4), 5,'Ground','FontSize',7,'Color','r','HorizontalAlignment','center');
text(t_wp(6), 5,'Right','FontSize',7,'HorizontalAlignment','center');
grid on;

subplot(3,2,2);
plot(t_sim, rad2deg(Qd), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (deg/s)');
title('Joint Velocities');
legend(joint_names,'Location','best','FontSize',8);
grid on;

subplot(3,2,3);
plot(t_sim, rad2deg(Qdd), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Accel (deg/s²)');
title('Joint Accelerations');
legend(joint_names,'Location','best','FontSize',8);
grid on;

subplot(3,2,4);
plot(t_sim, TAU, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Torque (N·m)');
title('Required Joint Torques');
legend(joint_names,'Location','best','FontSize',8);
xline(t_wp(4),'--r','Ground point','LabelVerticalAlignment','bottom');
grid on;

subplot(3,2,5);
b1 = bar(peak_tau, 'FaceColor', [0.2 0.5 0.9]);
hold on;
bar(peak_tau*1.5, 'FaceColor', 'none', 'EdgeColor', [0.9 0.2 0.2], ...
    'LineWidth', 1.5, 'LineStyle', '--');
set(gca,'XTickLabel',joint_names,'XTickLabelRotation',30);
ylabel('Torque (N·m)');
title('Peak Torque  (blue=actual, red dashed=×1.5 required)');
legend('Actual peak','Required (×1.5)','Location','northeast');
grid on;

subplot(3,2,6);
bar(cont_tau_rms, 'FaceColor', [0.9 0.5 0.2]);
set(gca,'XTickLabel',joint_names,'XTickLabelRotation',30);
ylabel('RMS Torque (N·m)');
title('Continuous (RMS) Torque per Joint');
grid on;

sgtitle('Worst Case: Full Stretch LEFT → Ground Arc → Full Stretch RIGHT  |  6s', ...
        'FontSize', 13);

% =========================================================
%  SECTION 8 — 3D ANIMATION
% =========================================================
figure('Name','Worst Case Animation','Color','w');

Kuka.plot(Q(1,:), 'workspace', [-0.6 0.6 -0.6 0.6 -0.3 0.7]);
hold on;

% Draw LEFT target (fully stretched left position)
theta_left = linspace(-pi/6, pi/6, 20);
xl = 0.30*cos(theta_left);
yl = 0.30*sin(theta_left) + 0.30;
fill3(xl, yl, zeros(size(xl)), [0.2 0.4 0.9], 'FaceAlpha', 0.4);
text(0, 0.30, 0.02, 'STRETCH LEFT', 'FontSize', 7, ...
     'HorizontalAlignment','center','Color',[0.2 0.4 0.9]);

% Draw GROUND ARC zone (front)
theta_arc = linspace(-pi/6, pi/6, 20);
xf = 0.28*cos(theta_arc);
yf = 0.28*sin(theta_arc);
fill3(xf, yf, zeros(size(xf)), [0.9 0.3 0.2], 'FaceAlpha', 0.4);
text(0.28, 0, 0.02, 'GROUND', 'FontSize', 7, ...
     'HorizontalAlignment','center','Color',[0.9 0.3 0.2]);

% Draw RIGHT target
xr = 0.30*cos(theta_left);
yr = 0.30*sin(theta_left) - 0.30;
fill3(xr, yr, zeros(size(xr)), [0.2 0.8 0.3], 'FaceAlpha', 0.4);
text(0, -0.30, 0.02, 'STRETCH RIGHT', 'FontSize', 7, ...
     'HorizontalAlignment','center','Color',[0.2 0.8 0.3]);

% Draw arc path preview
arc_angles  = linspace(pi/2, -pi/2, 100);
arc_r       = 0.30;
arc_x       = arc_r * cos(arc_angles);
arc_y       = arc_r * sin(arc_angles);
arc_z       = zeros(1,100);
plot3(arc_x, arc_y, arc_z, '--', 'Color', [0.5 0.5 0.5], ...
      'LineWidth', 1, 'DisplayName', 'Planned arc');

% Trajectory trail
h_traj    = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);
traj_hist = [];

fprintf('\nPlaying animation...\n');
skip = 4;
for k = 1:skip:N
    Kuka.animate(Q(k,:));
    T_now     = double(Kuka.fkine(Q(k,:)));
    traj_hist = [traj_hist, T_now(1:3,4)];
    set(h_traj, 'XData', traj_hist(1,:), ...
                'YData', traj_hist(2,:), ...
                'ZData', traj_hist(3,:));
    drawnow;
end

fprintf('\n========================================\n');
fprintf('  WORST CASE SIMULATION COMPLETE\n');
fprintf('  These are your MAXIMUM motor torque requirements\n');
fprintf('  Use peak × 1.5 for final motor selection\n');
fprintf('========================================\n');