%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

% Links Parameters
a1 = 37.2;
a2 = 138.10;
a3 = 28.2;
d1 = 135.80;
d2 = 160;
d3 = 15;

% Define the threshold for cleaning up small numbers
threshold = 1e-10;

%% --- ROBOT DEFINITION ---
% Link([theta, d, a, alpha])
% El le3b fel alpha ele ablaha bykhaly d ele ba3daha makanha yetghayar
L(1) = Link([0      d1     a1        pi/2]);            % R
L(2) = Link([pi/2   0      a2       -pi]);              % R
L(3) = Link([0      0      a3       -pi/2]);            % Ghayar L3 le d3
L(4) = Link([0      d2     0        pi/2]);             % R
L(5) = Link([0      0      0        -pi/2]);            % R
L(6) = Link([0      d3     0        0]);                % R

Rob = SerialLink(L, 'name', 'RRRRR');
q = [0 pi/2 0 0 0 0];
% q = [30*pi/180 -45*pi/180 0.2 60*pi/180 20*pi/180 90*pi/180];

% Plot the robot
Rob.plot(q, 'workspace', [-400 400 -400 400 -1 500]);

%% --- 1. INDIVIDUAL LINK TRANSFORMATIONS (A_i) ---
% This shows the transformation from link i-1 to link i (e.g., T_01, T_12, T_23...)

disp('--- Individual Link Transformations (A_i) ---');

% Get all individual A matrices (as a 4x4x6 matrix)
A_all = Rob.A(q); 

for i = 1:Rob.n
    fprintf('A%d (T_%d-%d):\n', i, i-1, i);
    
    % Get the individual matrix
    Ai = A_all(:,:,i);
    
    % Clean up small values for display
    Ai(abs(Ai) < threshold) = 0;
    disp(Ai);
end

%% --- 2. CUMULATIVE TRANSFORMATIONS (T_0i) ---
% This shows the transformation from the base (0) to link i (e.g., T_01, T_02, T_03...)

disp('--- Cumulative Link Transformations (T_0i) ---');

% Start with the identity matrix
T_cumulative = eye(4); 

for i = 1:Rob.n
    fprintf('T_0%d:\n', i);
    
    % Multiply by the next link's transform
    % We use A_all(:,:,i) from the previous step for the calculation
    T_cumulative = T_cumulative * A_all(:,:,i);
    
    % Create a copy to clean up for display
    T_display = T_cumulative;
    T_display(abs(T_display) < threshold) = 0;
    disp(T_display);
end

%% --- 3. FINAL TRANSFORMATION (T_06) ---
% This is the same as your original code and will match the last matrix
% printed in the section above (T_06).

disp('--- Final Transformation T_06 (from .fkine) ---');
T = Rob.fkine(q);

% Eliminate very small values from T (set values below threshold to 0)
T_matrix = double(T);  % Convert to double matrix if needed
T_matrix(abs(T_matrix) < threshold) = 0;
disp(T_matrix);

% Sliders
% Rob.teach;