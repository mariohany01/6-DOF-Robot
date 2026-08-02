% ---------------------------------------------------------
%  INVERSE JACOBIAN SOLVER WITH ANIMATION
% ---------------------------------------------------------
clc; clear; close all;

% --- 1. ROBOT SETUP ---
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;

L(1) = Link([0,     d1, a1,  pi/2]);
L(2) = Link([pi/2,   0, a2, -pi]);
L(3) = Link([0,      0, a3, -pi/2]);
L(4) = Link([0,     d2,  0,  pi/2]);
L(5) = Link([0,      0,  0, -pi/2]);
L(6) = Link([0,     d3,  0,  0]);
Kuka = SerialLink(L, 'name', 'My_Kuka');

% --- 2. SIMULATION SETTINGS ---
q_current = deg2rad([0, 90, 0, 0, 0, 0])';  % COLUMN vector now
V_desired = [0; 0; -40; 0; 0; 0];
dt = 0.05;
total_time = 3.0;
delta_math = 1e-6;

% --- 3. SETUP PLOT ---
figure('Name', 'Inverse Jacobian Animation', 'Color', 'w');
Kuka.plot(q_current', 'workspace', [-400 400 -400 400 -1 400]);
hold on;
h_traj = plot3(0,0,0, 'r.', 'MarkerSize', 5);
traj_hist = [];

% --- 4. ANIMATION LOOP ---
for t = 0:dt:total_time

    % STEP A: NUMERICAL JACOBIAN
    J_numerical = zeros(6,6);
    T0 = double(Kuka.fkine(q_current'));
    R0 = T0(1:3,1:3);
    P0 = T0(1:3,4);

    for i = 1:6
        q_pert = q_current;
        q_pert(i) = q_pert(i) + delta_math;
        T1 = double(Kuka.fkine(q_pert'));
        R1 = T1(1:3,1:3);
        P1 = T1(1:3,4);
        J_numerical(1:3, i) = (P1 - P0) / delta_math;
        dR = (R1 - R0) / delta_math;
        S = dR * R0';
        S_skew = 0.5 * (S - S');           % enforce skew-symmetry
        J_numerical(4:6, i) = Vex(S_skew);
    end

    % STEP B: SINGULARITY CHECK
    if cond(J_numerical) > 1e4
        warning('t=%.2f: Near singularity (cond=%.0f)', t, cond(J_numerical));
    end

    % STEP C: VELOCITY IK
    q_dot = pinv(J_numerical) * V_desired;

    % STEP D: SAFETY CLAMP (prevent runaway near singularity)
    max_qdot = 2.0; % rad/s limit per joint
    if max(abs(q_dot)) > max_qdot
        q_dot = q_dot * (max_qdot / max(abs(q_dot)));
    end

    % STEP E: INTEGRATE
    q_current = q_current + q_dot * dt;

    % STEP F: JOINT LIMITS ±180°
    q_current = max(min(q_current, pi), -pi);

    % STEP G: ANIMATE
    Kuka.animate(q_current');
    T_now = double(Kuka.fkine(q_current'));
    traj_hist = [traj_hist, T_now(1:3,4)];
    set(h_traj, 'XData', traj_hist(1,:), ...
                'YData', traj_hist(2,:), ...
                'ZData', traj_hist(3,:));
    drawnow;
end
disp('--- FINISHED ---');

% --- HELPER ---
function v = Vex(S)
    v = [S(3,2); S(1,3); S(2,1)];
end