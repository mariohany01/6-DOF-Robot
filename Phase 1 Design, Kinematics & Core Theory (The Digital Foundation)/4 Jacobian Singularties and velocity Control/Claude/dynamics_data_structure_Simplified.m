% =========================================================
%  ROBOT DYNAMICS - FAST VERSION (no simplify)
% =========================================================
clc; clear; close all;
syms q1 q2 q3 q4 q5 q6 real

q = [q1; q2; q3; q4; q5; q6];

% =========================================================
%  SECTION 1 — DH PARAMETERS
% =========================================================

% To these (meters):
a1 = 0.0375;  a2 = 0.1600; a3 = 0.0150;
d1 = 0.1358;  d2 = 0.1381; d3 = 0.0282;

%a1 = 37.5;  a2 = 160.0; a3 = 15.0;
%d1 = 135.8; d2 = 138.1; d3 = 28.2;

DH = [0,     d1, a1,  pi/2;
      pi/2,   0, a2,  -pi;
      0,      0, a3, -pi/2;
      0,     d2,  0,  pi/2;
      0,      0,  0, -pi/2;
      0,     d3,  0,  0];

% =========================================================
%  SECTION 2 — MASS PROPERTIES
%  Units: kg, meters, kg*m²
% =========================================================
g2kg        = 1e-3;
mm2m        = 1e-3;
gmm2_kgm2   = 1e-9;

m(1) = 1010.74 * g2kg;
rc{1} = [-57.76; -5.77; -23.13] * mm2m;
I{1}  = [1379194.99, -96653.51,  472553.44;
          -96653.51, 2446919.81,  -97922.48;
          472553.44,  -97922.48, 1886347.08] * gmm2_kgm2;

m(2) = 687.67 * g2kg;
rc{2} = [-90.56; 0.00; -50.83] * mm2m;
I{2}  = [ 384127.81,     -23.76, -206242.83;
              -23.76, 1913642.15,      23.52;
         -206242.83,      23.52, 1825142.79] * gmm2_kgm2;

m(3) = 437.93 * g2kg;
rc{3} = [-18.68; 5.37; -10.61] * mm2m;
I{3}  = [336996.04,  17757.31,  78065.99;
          17757.31, 406643.04,  38017.26;
          78065.99,  38017.26, 322130.64] * gmm2_kgm2;

m(4) = 173.56 * g2kg;
rc{4} = [-0.03; -193.39; 0.00] * mm2m;
I{4}  = [453606.15,      1.37,     -0.07;
               1.37,  26509.07,    -10.05;
              -0.07,    -10.05, 451259.47] * gmm2_kgm2;

m(5) = 168.27 * g2kg;
rc{5} = [0.01; -4.75; -44.47] * mm2m;
I{5}  = [214783.40,    -22.29,    -72.40;
             -22.29, 183460.47,  -4944.94;
             -72.40,  -4944.94,  63473.33] * gmm2_kgm2;

m(6) = 63.68 * g2kg;
rc{6} = [0.00; -2.65; -24.44] * mm2m;
I{6}  = [12822.27,    0.00,    0.02;
              0.00, 6988.11,  633.04;
              0.02,  633.04, 9882.98] * gmm2_kgm2;

fprintf('Total mass: %.3f kg\n', sum(m));

% =========================================================
%  SECTION 3 — SYMBOLIC TRANSFORMS (no simplify)
% =========================================================
fprintf('Building transforms...\n');

theta_offset = DH(:,1);

A = cell(6,1);
for i = 1:6
    th = q(i) + theta_offset(i);
    d  = DH(i,2);
    a  = DH(i,3);
    al = DH(i,4);
    A{i} = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
            sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
            0,         sin(al),          cos(al),         d;
            0,         0,                0,               1];
end

T = cell(6,1);
T{1} = A{1};
for i = 2:6
    T{i} = T{i-1} * A{i};   % NO simplify — this is the key change
    fprintf('  T{%d} done\n', i);
end

% =========================================================
%  SECTION 4 — PER-LINK JACOBIANS (symbolic, no simplify)
% =========================================================
fprintf('Building Jacobians...\n');

%Building constants
z0 = [0;0;1];
p0 = [0;0;0];

Jv = cell(6,1);
Jw = cell(6,1);

%CHOOSING LINK FROM 1-->6
for i = 1:6
    Jv{i} = sym(zeros(3,6));
    Jw{i} = sym(zeros(3,6));

    R_i      = T{i}(1:3,1:3);
    p_i      = T{i}(1:3,4);
    rc_base  = p_i + R_i * rc{i};

%How Fast COM Of Link i Moves when Joint j Moves
    for j = 1:6
        if j == 1
            z_prev = z0;
            p_prev = p0;
        else
            z_prev = T{j-1}(1:3,3);
            p_prev = T{j-1}(1:3,4);
        end

        if j <= i
            Jv{i}(:,j) = cross(z_prev, rc_base - p_prev);
            Jw{i}(:,j) = z_prev;
        end
    end
    fprintf('  Link %d Jacobian done\n', i);
end

% =========================================================
%  SECTION 5 — MASS MATRIX M(q) symbolic then export
% =========================================================
fprintf('Building M(q) symbolically...\n');

M_sym = sym(zeros(6,6));
for i = 1:6
    R_i    = T{i}(1:3,1:3);
    I_base = R_i * I{i} * R_i';
    M_sym  = M_sym + m(i)*(Jv{i}'*Jv{i}) + Jw{i}'*I_base*Jw{i};
end
% Single collect at the end — much faster than per-step simplify
M_sym = collect(M_sym, q);
fprintf('  M(q) done\n');

% =========================================================
%  SECTION 6 — GRAVITY VECTOR G(q) symbolic then export
% =========================================================
fprintf('Building G(q)...\n');

grav    = 9.81;
P_total = sym(0);
for i = 1:6
    R_i     = T{i}(1:3,1:3);
    p_i     = T{i}(1:3,4);
    rc_base = p_i + R_i * rc{i};
    P_total = P_total + m(i) * grav * rc_base(3);
end

G_sym = sym(zeros(6,1));
for j = 1:6
    G_sym(j) = diff(P_total, q(j));
end
fprintf('  G(q) done\n');

% =========================================================
%  SECTION 7 — EXPORT M and G only
%  Coriolis C is computed NUMERICALLY at runtime (see below)
% =========================================================
fprintf('Exporting get_M and get_G...\n');

args_q = {q1,q2,q3,q4,q5,q6};
matlabFunction(M_sym, 'File', 'get_M', 'Vars', args_q, 'Optimize', true);
matlabFunction(G_sym, 'File', 'get_G', 'Vars', args_q, 'Optimize', true);

fprintf('Done. Saved: get_M.m and get_G.m\n');
fprintf('Expected runtime: 3-8 minutes total\n');

% =========================================================
%  SECTION 8 — NUMERICAL CORIOLIS FUNCTION
%  Save this as get_C.m in your working folder
% =========================================================
% C is computed via numerical differentiation of M:
%   C_kj = sum_i 0.5*(dMkj/dqi + dMki/dqj - dMij/dqk) * dqi
%
% Copy the function below into a NEW file called get_C.m

fprintf('\nNow create get_C.m manually — see code below\n');
fprintf('Or run: edit get_C.m and paste the function\n');