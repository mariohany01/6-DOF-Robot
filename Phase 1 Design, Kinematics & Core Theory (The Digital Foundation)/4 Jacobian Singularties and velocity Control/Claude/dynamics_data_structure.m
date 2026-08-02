% =========================================================
%  ROBOT DYNAMICS - COMPLETE SETUP
%  Loads mass properties and builds M(q), C(q,qdot), G(q)
% =========================================================
clc; clear; close all;
syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real

q  = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

% =========================================================
%  SECTION 1 — DH PARAMETERS
% =========================================================
a1 = 37.5;  a2 = 160.0; a3 = 15.0;
d1 = 135.8; d2 = 138.1; d3 = 28.2;

% DH table rows: [theta_offset, d, a, alpha]
DH = [0,      d1, a1,  pi/2;
      pi/2,    0, a2, -pi;
      0,       0, a3, -pi/2;
      0,      d2,  0,  pi/2;
      0,       0,  0, -pi/2;
      0,      d3,  0,  0];

% =========================================================
%  SECTION 2 — MASS PROPERTIES (from your SolidWorks data)
%  Units: mass in kg, CoM in meters, Inertia in kg*m^2
% =========================================================

% --- Conversion factors ---
g2kg   = 1e-3;       % grams  → kg
mm2m   = 1e-3;       % mm     → m
gmm2_2_kgm2 = 1e-9; % g*mm²  → kg*m²

% --- Link 1 (Joint 01) ---
m(1) = 1010.74 * g2kg;
rc{1} = [-57.76; -5.77; -23.13] * mm2m;   % CoM in local DH frame
I{1}  = [1379194.99, -96653.51,  472553.44;
         -96653.51, 2446919.81,  -97922.48;
          472553.44, -97922.48, 1886347.08] * gmm2_2_kgm2;

% --- Link 2 (Joint 02) ---
m(2) = 687.67 * g2kg;
rc{2} = [-90.56; 0.00; -50.83] * mm2m;
I{2}  = [384127.81,    -23.76, -206242.83;
            -23.76, 1913642.15,      23.52;
        -206242.83,      23.52, 1825142.79] * gmm2_2_kgm2;

% --- Link 3 (Joint 03) ---
m(3) = 437.93 * g2kg;
rc{3} = [-18.68; 5.37; -10.61] * mm2m;
I{3}  = [336996.04,  17757.31,  78065.99;
          17757.31, 406643.04,  38017.26;
          78065.99,  38017.26, 322130.64] * gmm2_2_kgm2;

% --- Link 4 (Joint 04) ---
m(4) = 173.56 * g2kg;
rc{4} = [-0.03; -193.39; 0.00] * mm2m;
I{4}  = [453606.15,      1.37,     -0.07;
               1.37,  26509.07,    -10.05;
              -0.07,    -10.05, 451259.47] * gmm2_2_kgm2;

% --- Link 5 (Joint 05) ---
m(5) = 168.27 * g2kg;
rc{5} = [0.01; -4.75; -44.47] * mm2m;
I{5}  = [214783.40,    -22.29,    -72.40;
            -22.29, 183460.47,  -4944.94;
            -72.40,  -4944.94,  63473.33] * gmm2_2_kgm2;

% --- Link 6 (Joint 06) ---
m(6) = 63.68 * g2kg;
rc{6} = [0.00; -2.65; -24.44] * mm2m;
I{6}  = [12822.27,     0.00,    0.02;
              0.00,  6988.11,  633.04;
              0.02,   633.04, 9882.98] * gmm2_2_kgm2;

fprintf('Total robot mass: %.3f kg\n', sum(m));

% =========================================================
%  SECTION 3 — HOMOGENEOUS TRANSFORMATION MATRICES
%  Build A_i (frame i-1 to frame i) symbolically
% =========================================================
fprintf('Building transformation matrices...\n');

% Joint angles including DH offsets
theta = [q1 + DH(1,1);
         q2 + DH(2,1);
         q3 + DH(3,1);
         q4 + DH(4,1);
         q5 + DH(5,1);
         q6 + DH(6,1)];

% Build individual A matrices
A = cell(6,1);
for i = 1:6
    th = theta(i);
    d  = DH(i,2);
    a  = DH(i,3);
    al = DH(i,4);

    A{i} = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
            sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
            0,        sin(al),           cos(al),         d;
            0,        0,                 0,               1];
end

% Build cumulative transforms T{i} = A{1}*...*A{i}
% T{i} = transform from base to frame i
T = cell(6,1);
T{1} = A{1};
for i = 2:6
    T{i} = simplify(T{i-1} * A{i});
    fprintf('  T{%d} done\n', i);
end

% =========================================================
%  SECTION 4 — PER-LINK JACOBIANS
%  Jv_i = linear velocity Jacobian for CoM of link i
%  Jw_i = angular velocity Jacobian for link i
% =========================================================
fprintf('Building per-link Jacobians...\n');

% Base frame z-axis and origin
z0 = [0; 0; 1];
p0 = [0; 0; 0];

Jv = cell(6,1);
Jw = cell(6,1);

for i = 1:6
    Jv{i} = sym(zeros(3,6));
    Jw{i} = sym(zeros(3,6));

    % CoM position of link i in base frame
    R_i  = T{i}(1:3,1:3);
    p_i  = T{i}(1:3,4);
    rc_base = p_i + R_i * rc{i};  % CoM in base frame

    for j = 1:6
        if j == 1
            % Joint j=1: use base frame z-axis and origin
            z_prev = z0;
            p_prev = p0;
        else
            % Joint j: z-axis and origin from frame j-1
            T_prev = T{j-1};
            z_prev = T_prev(1:3,3);
            p_prev = T_prev(1:3,4);
        end

        if j <= i
            % Joint j affects link i
            % Linear: z_{j-1} × (rc_i - p_{j-1})
            Jv{i}(:,j) = cross(z_prev, rc_base - p_prev);
            % Angular: z_{j-1}
            Jw{i}(:,j) = z_prev;
        else
            % Joint j does NOT affect link i
            Jv{i}(:,j) = [0;0;0];
            Jw{i}(:,j) = [0;0;0];
        end
    end
    fprintf('  Jacobian for link %d done\n', i);
end

% =========================================================
%  SECTION 5 — MASS MATRIX M(q)
%  M = sum_i [ m_i * Jv_i' * Jv_i + Jw_i' * R_i * I_i * R_i' * Jw_i ]
% =========================================================
fprintf('Computing mass matrix M(q)... (this takes a few minutes)\n');

M_sym = sym(zeros(6,6));
for i = 1:6
    R_i   = T{i}(1:3,1:3);
    % Rotate inertia tensor to base frame
    I_base = R_i * I{i} * R_i';
    M_sym  = M_sym + m(i) * (Jv{i}' * Jv{i}) ...
                   + Jw{i}' * I_base * Jw{i};
end
M_sym = simplify(M_sym);
fprintf('  M(q) done\n');

% =========================================================
%  SECTION 6 — GRAVITY VECTOR G(q)
%  G_j = dP/dq_j where P = sum_i m_i * g * h_i
%  g pointing in -Z of base frame
% =========================================================
fprintf('Computing gravity vector G(q)...\n');

grav = 9.81; % m/s^2

P_total = sym(0);
for i = 1:6
    R_i     = T{i}(1:3,1:3);
    p_i     = T{i}(1:3,4);
    rc_base = p_i + R_i * rc{i};
    % Height = Z component of CoM in base frame
    h_i     = rc_base(3);
    P_total = P_total + m(i) * grav * h_i;
end

G_sym = sym(zeros(6,1));
for j = 1:6
    G_sym(j) = diff(P_total, q(j));
end
G_sym = simplify(G_sym);
fprintf('  G(q) done\n');

% =========================================================
%  SECTION 7 — CORIOLIS MATRIX C(q, dq)
%  Using Christoffel symbols:
%  C_kj = sum_i [ (dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k)/2 ] * dq_i
% =========================================================
fprintf('Computing Coriolis matrix C(q,dq)... (slowest step)\n');

C_sym = sym(zeros(6,6));
for k = 1:6
    for j = 1:6
        c_kj = sym(0);
        for i = 1:6
            c_kj = c_kj + (diff(M_sym(k,j), q(i)) + ...
                           diff(M_sym(k,i), q(j)) - ...
                           diff(M_sym(i,j), q(k))) / 2 * dq(i);
        end
        C_sym(k,j) = c_kj;
    end
end
C_sym = simplify(C_sym);
fprintf('  C(q,dq) done\n');

% =========================================================
%  SECTION 8 — EXPORT TO MATLAB FUNCTIONS
%  Converts symbolic expressions to fast numeric functions
% =========================================================
fprintf('Exporting to MATLAB functions...\n');

args_q   = {q1,q2,q3,q4,q5,q6};
args_qdq = {q1,q2,q3,q4,q5,q6, dq1,dq2,dq3,dq4,dq5,dq6};

matlabFunction(M_sym, 'File', 'get_M', 'Vars', args_q);
matlabFunction(C_sym, 'File', 'get_C', 'Vars', args_qdq);
matlabFunction(G_sym, 'File', 'get_G', 'Vars', args_q);

fprintf('Done. Functions saved: get_M.m, get_C.m, get_G.m\n');
fprintf('\nTo use them:\n');
fprintf('  M = get_M(q(1),q(2),q(3),q(4),q(5),q(6))\n');
fprintf('  C = get_C(q(1),...,q(6),dq(1),...,dq(6))\n');
fprintf('  G = get_G(q(1),q(2),q(3),q(4),q(5),q(6))\n');
fprintf('  tau = M*ddq + C*dq + G\n');