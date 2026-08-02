% ---------------------------------------------------------
%  INVERSE JACOBIAN SOLVER — COMPUTE THEN ANIMATE
% ---------------------------------------------------------
clc; clear; close all;

% --- 1. ROBOT SETUP ---
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;
L(1) = Link([0,    d1, a1,  pi/2]);
L(2) = Link([pi/2,  0, a2,    -pi]);
L(3) = Link([0,     0, a3, -pi/2]);
L(4) = Link([0,    d2,  0,  pi/2]);
L(5) = Link([0,     0,  0, -pi/2]);
L(6) = Link([0,    d3,  0,     0]);
Kuka = SerialLink(L, 'name', 'My_Kuka');

% --- 2. SETTINGS ---
q_current  = deg2rad([0, 90, 0, 0, 0, 0]);
V_desired  = [0; 0; -40; 0; 0; 0];
dt         = 0.005;
total_time = 3.0;
delta_math = 1e-6;
steps      = round(total_time / dt);   % = 600

% =====================================================
%  PHASE 1 — COMPUTE ALL STEPS, SAVE HISTORY
% =====================================================
q_history   = zeros(steps, 6);   % 600 rows × 6 joints
pos_history = zeros(3, steps);   % x,y,z at each step

disp('--- COMPUTING ... ---');

for k = 1:steps

    % -- Jacobian --
    J = zeros(6,6);
    T0 = double(Kuka.fkine(q_current));
    R0 = T0(1:3,1:3);
    P0 = T0(1:3,4);

    for i = 1:6
        q_pert    = q_current;
        q_pert(i) = q_pert(i) + delta_math;
        T1 = double(Kuka.fkine(q_pert));
        J(1:3, i) = (T1(1:3,4) - P0) / delta_math;
        dR        = (T1(1:3,1:3) - R0) / delta_math;
        J(4:6, i) = Vex(dR * R0');
    end

    % -- IK --
    q_dot     = pinv(J) * V_desired;

    % -- Integrate --
    q_current = q_current + (q_dot' * dt);

    % -- Save --
    q_history(k, :)   = q_current;
    pos_history(:, k) = T0(1:3, 4);   % tip position this step

end

disp('--- COMPUTING DONE. STARTING ANIMATION ---');

% =====================================================
%  PHASE 2 — ANIMATE FROM SAVED HISTORY
% =====================================================
figure('Name', 'Inverse Jacobian Animation', 'Color', 'w');
Kuka.plot(q_history(1,:), 'workspace', [-400 400 -400 400 -1 400]);
hold on;
h_traj = plot3(0, 0, 0, 'r.', 'MarkerSize', 5);
traj_hist = [];

for k = 1:steps
    Kuka.animate(q_history(k, :));

    traj_hist = [traj_hist, pos_history(:, k)];
    set(h_traj, 'XData', traj_hist(1,:), ...
                'YData', traj_hist(2,:), ...
                'ZData', traj_hist(3,:));

    pause(dt);   % <-- this controls animation speed, not CPU
    drawnow;
end

disp('--- FINISHED ---');

% --- HELPER ---
function v = Vex(S)
    v = [0.5*(S(3,2)-S(2,3));
         0.5*(S(1,3)-S(3,1));
         0.5*(S(2,1)-S(1,2))];
end