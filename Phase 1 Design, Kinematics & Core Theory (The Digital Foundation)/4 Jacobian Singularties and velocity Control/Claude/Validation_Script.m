% =========================================================
%  DYNAMICS VALIDATION SCRIPT
%  Tests M(q) and G(q) for physical correctness
% =========================================================
clc; clear;

fprintf('========================================\n');
fprintf('  DYNAMICS VALIDATION\n');
fprintf('========================================\n\n');

% =========================================================
%  TEST 1 — M(q) must be Symmetric & Positive Definite
%  Physical meaning: kinetic energy = 0.5*dq'*M*dq > 0 always
% =========================================================
fprintf('--- TEST 1: M(q) Symmetry & Positive Definiteness ---\n');

q_home = deg2rad([0, 90, 0, 0, 0, 0]);

M = get_M(q_home(1),q_home(2),q_home(3), ...
          q_home(4),q_home(5),q_home(6));

% Symmetry check
sym_error = max(max(abs(M - M')));
fprintf('Symmetry error (should be < 1e-10): %.2e  ', sym_error);
if sym_error < 1e-10
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end

% Positive definiteness — all eigenvalues must be > 0
eigs_M = eig(M);
fprintf('Eigenvalues of M (all must be > 0):\n');
disp(eigs_M');
if all(eigs_M > 0)
    fprintf('Positive Definite: PASS\n');
else
    fprintf('Positive Definite: FAIL — check inertia tensors\n');
end

% Print M for inspection
fprintf('\nM(q_home) matrix [kg*m^2]:\n');
disp(M);

% =========================================================
%  TEST 2 — G(q) gravity sanity check
%  At home pose [0,90,0,0,0,0] gravity torques must make
%  physical sense — joint 2 should carry the most load
% =========================================================
fprintf('\n--- TEST 2: G(q) Gravity Torques ---\n');

G = get_G(q_home(1),q_home(2),q_home(3), ...
          q_home(4),q_home(5),q_home(6));

fprintf('Gravity torques at home position [N*m]:\n');
joint_names = {'Joint 1 (base rotate)', ...
               'Joint 2 (shoulder)', ...
               'Joint 3 (elbow)', ...
               'Joint 4 (wrist 1)', ...
               'Joint 5 (wrist 2)', ...
               'Joint 6 (wrist 3)'};
for i = 1:6
    fprintf('  %s:  %.4f N*m\n', joint_names{i}, G(i));
end

% Physical check: joint 1 (base) gravity torque should be ~0
% because rotating about vertical Z axis doesn't fight gravity
fprintf('\nPhysical checks:\n');
if abs(G(1)) < 0.1
    fprintf('  Joint 1 gravity torque ≈ 0: PASS (base spins about Z)\n');
else
    fprintf('  Joint 1 gravity torque = %.4f: WARNING (expected ~0)\n', G(1));
end

% Joint 2 should have largest gravity torque (holds the whole arm)
[~, max_idx] = max(abs(G));
fprintf('  Largest gravity torque at Joint %d ', max_idx);
if max_idx == 2 || max_idx == 3
    fprintf('(Joint 2 or 3): PASS\n');
else
    fprintf(': WARNING — expected Joint 2 or 3 to dominate\n');
end

% =========================================================
%  TEST 3 — G(q) at flat pose [0,0,0,0,0,0]
%  All joints horizontal — gravity effect should be different
% =========================================================
fprintf('\n--- TEST 3: G(q) at flat pose [0,0,0,0,0,0] ---\n');

q_flat = zeros(1,6);
G_flat = get_G(q_flat(1),q_flat(2),q_flat(3), ...
               q_flat(4),q_flat(5),q_flat(6));

fprintf('Gravity torques at flat position [N*m]:\n');
for i = 1:6
    fprintf('  Joint %d:  %.4f N*m\n', i, G_flat(i));
end

% =========================================================
%  TEST 4 — Energy consistency check
%  KE = 0.5 * dq' * M * dq must be positive for any dq
% =========================================================
fprintf('\n--- TEST 4: Kinetic Energy Positivity ---\n');

test_poses = {deg2rad([0,90,0,0,0,0]), ...
              deg2rad([45,45,30,0,0,0]), ...
              deg2rad([90,60,-30,45,0,0])};
pose_names = {'Home [0,90,0,0,0,0]', ...
              'Mid  [45,45,30,0,0,0]', ...
              'Ext  [90,60,-30,45,0,0]'};
dq_test = [1;0.5;0.3;0.2;0.1;0.05];  % arbitrary velocity

all_pass = true;
for k = 1:3
    qk = test_poses{k};
    Mk = get_M(qk(1),qk(2),qk(3),qk(4),qk(5),qk(6));
    KE = 0.5 * dq_test' * Mk * dq_test;
    pass = KE > 0;
    fprintf('  %s — KE = %.6f J  %s\n', ...
            pose_names{k}, KE, ternary(pass,'PASS','FAIL'));
    all_pass = all_pass && pass;
end
if all_pass
    fprintf('All kinetic energy checks: PASS\n');
end

% =========================================================
%  TEST 5 — C(q,dq) skew-symmetry check
%  (M_dot - 2C) must be skew-symmetric — passivity property
%  This validates get_C.m
% =========================================================
fprintf('\n--- TEST 5: Coriolis Passivity Check ---\n');

q0  = deg2rad([0, 90, 0, 0, 0, 0]);
dq0 = deg2rad([10, 5, 3, 2, 1, 0.5]);

C0  = get_C(q0, dq0);

% Compute M_dot numerically
dt  = 1e-5;
q1s = q0 + dq0 * dt;
M1  = get_M(q1s(1),q1s(2),q1s(3),q1s(4),q1s(5),q1s(6));
M0  = get_M(q0(1), q0(2), q0(3), q0(4), q0(5), q0(6));
Mdot = (M1 - M0) / dt;

% Check: dq' * (Mdot - 2C) * dq should = 0 (passivity)
N    = Mdot - 2*C0;
passivity_val = dq0 * N * dq0';
fprintf('Passivity check dq*(Mdot-2C)*dq (should be ≈ 0): %.6f\n', passivity_val);
if abs(passivity_val) < 0.01
    fprintf('Passivity: PASS\n');
else
    fprintf('Passivity: WARNING — value = %.4f (check get_C.m)\n', passivity_val);
end

% =========================================================
%  SUMMARY
% =========================================================
fprintf('\n========================================\n');
fprintf('  Copy your results and share them\n');
fprintf('  All 5 tests should show PASS\n');
fprintf('========================================\n');

% Helper
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end