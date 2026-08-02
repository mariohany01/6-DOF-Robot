% ---- JACOBIAN VALIDATION - LEVEL 1 ----
clc; clear;

% Robot setup
a1 = 37.5;  a2 = 160;  a3 = 15;
d1 = 135.8; d2 = 138.1; d3 = 28.2;
L(1) = Link([0,    d1, a1,  pi/2]);
L(2) = Link([pi/2,  0, a2, -pi]);
L(3) = Link([0,     0, a3, -pi/2]);
L(4) = Link([0,    d2,  0,  pi/2]);
L(5) = Link([0,     0,  0, -pi/2]);
L(6) = Link([0,    d3,  0,  0]);
Kuka = SerialLink(L, 'name', 'Kuka');

%%
%--- TEST 1: Sanity Check at Home Position ---

q_test = deg2rad([0, 90, 0, 0, 0, 0]);

% Your numerical Jacobian
J_num = numerical_jacobian(Kuka, q_test, 1e-6);

% Toolbox built-in (ground truth)
J_toolbox = Kuka.jacob0(q_test);

% Display both
disp('=== YOUR Numerical Jacobian ===');
disp(round(J_num, 4));

disp('=== TOOLBOX Jacobian (ground truth) ===');
disp(round(J_toolbox, 4));

% Error check
max_error = max(max(abs(J_num - J_toolbox)));
fprintf('\nMax element-wise error: %.6f\n', max_error);

if max_error < 1e-3
    disp('PASS — Jacobians match');
else
    disp('FAIL — mismatch detected');
    disp('Error matrix:');
    disp(J_num - J_toolbox);
end

%%
% --- TEST 2: Physical interpretation of each column ---
fprintf('\n=== COLUMN-BY-COLUMN PHYSICAL CHECK ===\n');
fprintf('Each column = effect of moving ONE joint on end-effector\n\n');

joint_names = {'Base rotation', 'Shoulder', 'Elbow', ...
               'Wrist 1', 'Wrist 2', 'Wrist 3'};

for i = 1:6
    lin_mag  = norm(J_num(1:3, i));
    ang_mag  = norm(J_num(4:6, i));
    fprintf('Joint %d (%s):\n', i, joint_names{i});
    fprintf('  Linear  effect magnitude:  %.2f mm per rad\n', lin_mag);
    fprintf('  Angular effect magnitude:  %.4f rad per rad\n', ang_mag);

    % Check: wrist joints (4,5,6) should have small LINEAR effect
    if i >= 4 && lin_mag > 50
        fprintf('  WARNING: Wrist joint has large linear effect (%.1f) - check offsets\n', lin_mag);
    end
    % Check: angular magnitude should always be 1.0 for revolute joints
    if abs(ang_mag - 1.0) > 0.01
        fprintf('  WARNING: Angular magnitude = %.4f (expected ~1.0)\n', ang_mag);
    end
end

%%
% --- TEST 3: Verify J*q_dot matches actual FK velocity ---
fprintf('\n=== FORWARD VALIDATION: J*q_dot vs actual FK difference ===\n');

q0   = deg2rad([0, 90, 0, 0, 0, 0]);
dt   = 1e-4;                          % tiny time step
q_dot_test = deg2rad([10, 0, 0, 0, 0, 0]); % only joint 1 moving at 10 deg/s

% Predicted end-effector velocity from Jacobian
V_predicted = J_num * q_dot_test';

% Actual end-effector displacement from FK
T0   = double(Kuka.fkine(q0));
T1   = double(Kuka.fkine(q0 + q_dot_test * dt));
V_actual_lin  = (T1(1:3,4) - T0(1:3,4)) / dt;

fprintf('Predicted linear velocity (J*qdot): [%.3f  %.3f  %.3f] mm/s\n', V_predicted(1:3));
fprintf('Actual    linear velocity (FK diff): [%.3f  %.3f  %.3f] mm/s\n', V_actual_lin);
fprintf('Error: [%.6f  %.6f  %.6f]\n', V_predicted(1:3) - V_actual_lin);

% Repeat for joint 3 (elbow)
q_dot_test2 = deg2rad([0, 0, 20, 0, 0, 0]);
V_predicted2 = J_num * q_dot_test2';
T2 = double(Kuka.fkine(q0 + q_dot_test2 * dt));
V_actual2    = (T2(1:3,4) - T0(1:3,4)) / dt;
fprintf('\nJoint 3 test:\n');
fprintf('Predicted: [%.3f  %.3f  %.3f]\n', V_predicted2(1:3));
fprintf('Actual:    [%.3f  %.3f  %.3f]\n', V_actual2);

%%
% ---- HELPER FUNCTION (must be at bottom of script) ----
function J = numerical_jacobian(robot, q, delta)
    J = zeros(6,6);
    T0 = double(robot.fkine(q));
    R0 = T0(1:3,1:3);
    P0 = T0(1:3,4);
    for i = 1:6
        q_pert    = q;
        q_pert(i) = q_pert(i) + delta;
        T1        = double(robot.fkine(q_pert));
        R1        = T1(1:3,1:3);
        P1        = T1(1:3,4);
        J(1:3, i) = (P1 - P0) / delta;
        dR        = (R1 - R0) / delta;
        S         = dR * R0';
        S_skew    = 0.5*(S - S');
        J(4:6, i) = [S_skew(3,2); S_skew(1,3); S_skew(2,1)];
    end
end
