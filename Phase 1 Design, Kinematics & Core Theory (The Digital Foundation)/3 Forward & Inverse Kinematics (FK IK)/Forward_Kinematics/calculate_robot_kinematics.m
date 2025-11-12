function [T_final_simple, euler_ZYX, fk_map] = calculate_robot_kinematics()
% calculate_robot_kinematics Performs symbolic forward kinematics for a 6-DOF arm.
%
% OUTPUTS:
%   T_final_simple - The 4x4 symbolic simplified transformation matrix.
%   euler_ZYX      - A struct containing the symbolic ZYX Euler angles.
%   fk_map         - A dictionary mapping element names ('r11', 'px', etc.)
%                    to their symbolic expressions.

%% --- 1. Define Symbolic Variables ---
syms theta1 theta2 theta3 theta4 theta5 theta6  % Joint variables (simpler names)
syms a1 a2 a3 d1 d2 d3                         % Link parameters

%% --- 2. Define DH Parameters ---
% The DH parameter table: [theta, d, a, alpha]
dh_table = [
    theta1,   d1,   a1,   pi/2;
    theta2,   0,    a2,   -pi;
    theta3,   0,    a3,   -pi/2;
    theta4,   d2,   0,    pi/2;
    theta5,   0,    0,    -pi/2;
    theta6,   d3,   0,    0
];

%% --- 3. Calculate Forward Kinematics ---
% Call the helper function to get the total transformation matrix
T_final = manual_dh_matrix(dh_table);

%% --- 4. Simplify the Result ---
% This is a crucial step for symbolic math
T_final_simple = simplify(T_final);

%% --- 5. Extract Elements into a Map (The "Clean" Way) ---
% No loops, no dummy variables. Just create the keys and values directly.
T = T_final_simple; % Use a short alias for convenience

keys = [
    "r11", "r12", "r13", "px", ...
    "r21", "r22", "r23", "py", ...
    "r31", "r32", "r33", "pz"
];

values = [
    T(1,1), T(1,2), T(1,3), T(1,4), ...
    T(2,1), T(2,2), T(2,3), T(2,4), ...
    T(3,1), T(3,2), T(3,3), T(3,4)
];

fk_map = dictionary(keys, values);

%% --- 6. Calculate ZYX Euler Angles ---
% We can access the map, or just use the matrix elements directly.
% Using the map makes the formulas very readable.

% Yaw-Pitch-Roll (ZYX) convention
yaw   = atan2(fk_map('r23'), fk_map('r13'));
pitch = atan2(sqrt(1 - fk_map('r33')^2), fk_map('r33'));
roll  = atan2(fk_map('r32'), -fk_map('r31'));

% Store in a struct for a clean return value
euler_ZYX = struct('Yaw_Z', yaw, 'Pitch_Y', pitch, 'Roll_X', roll);

end