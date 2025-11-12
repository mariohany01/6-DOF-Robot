%% --- SETUP ---
clc; clear; close all;

% Links Parameters
a1 = 37.2;
a2 = 138.10;
a3 = 28.2;
d1 = 135.80;
d2 = 160;
d3 = 15;

threshold = 1e-10;

%% --- ROBOT DEFINITION ---
% Standard DH: Link([theta, d, a, alpha])
% If you want a standard Revolute joint, theta is the variable (leave as 0 or offset).

L(1) = Link([0      d1     a1        pi/2]); 
L(2) = Link([pi/2   0      a2       -pi]);   % Note: pi/2 is a constant offset here
L(3) = Link([0      0      a3       -pi/2]); % REVOLUTE. (If you want prismatic, add 1 at the end)
L(4) = Link([0      d2     0        pi/2]);
L(5) = Link([0      0      0        -pi/2]);
L(6) = Link([0      d3     0        0]);     % d3 is the offset here

% Updated name to reflect 6 Links
Rob = SerialLink(L, 'name', '6-DOF Robot');

% Robot Configuration
q = [0, pi/2, 0, 0, 0, 0];

% Plot
figure(1);
Rob.plot(q, 'scale', 0.5, 'workspace', [-300 300 -300 300 0 400]);

%% --- 1. INDIVIDUAL LINK TRANSFORMATIONS (A_i) ---
disp('--- Individual Link Transformations (A_i) ---');

% We cannot calculate A_all at once. We must loop through links.
for i = 1:Rob.n
    fprintf('A%d (T_%d-%d):\n', i, i-1, i);
    
    % CORRECTED LINE: Calculate A for the specific link using its specific angle q(i)
    Ai = L(i).A(q(i)); 
    
    % Convert to double to handle symbolic/SE3 types
    Ai_mat = double(Ai);
    
    % Clean up
    Ai_mat(abs(Ai_mat) < threshold) = 0;
    disp(Ai_mat);
    
    % Store for next step
    A_stack(:,:,i) = Ai_mat;
end

%% --- 2. CUMULATIVE TRANSFORMATIONS (T_0i) ---
disp('--- Cumulative Link Transformations (T_0i) ---');

T_cumulative = eye(4); 

for i = 1:Rob.n
    fprintf('T_0%d:\n', i);
    
    % Multiply current cumulative by the specific link transform stored previously
    T_cumulative = T_cumulative * A_stack(:,:,i);
    
    % Clean up
    T_display = T_cumulative;
    T_display(abs(T_display) < threshold) = 0;
    disp(T_display);
end

%% --- 3. FINAL TRANSFORMATION (T_06) ---
disp('--- Final Transformation T_06 (from .fkine) ---');

% fkine calculates T_06 automatically
T = Rob.fkine(q);

% Convert SE3 object to 4x4 matrix
T_matrix = double(T);
T_matrix(abs(T_matrix) < threshold) = 0;
disp(T_matrix);

% Rob.teach; % Uncomment to use sliders