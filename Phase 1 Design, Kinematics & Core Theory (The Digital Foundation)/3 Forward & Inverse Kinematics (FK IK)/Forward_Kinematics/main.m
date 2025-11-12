% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

fprintf('Running 6-DOF Robot Kinematic Analysis...\n\n');

%% Symbolic Values 

% --- EXECUTE ---
% Call the main function to get all calculated symbolic data
[T_final, euler_angles, kinematics_map] = calculate_robot_kinematics();

% --- DISPLAY RESULTS ---
fprintf('==============================================================================================================================================================================\n');
fprintf('Simplified End-Effector Transformation Matrix:\n');
fprintf('==================================================\n');
disp(T_final);

fprintf('\n==================================================\n');
fprintf('ZYX Euler Angles (Yaw, Pitch, Roll):\n');
fprintf('==================================================\n');
disp(euler_angles);

fprintf('\n==================================================\n');
fprintf('Sample Element Access map:\n');
fprintf('==================================================\n');
disp(kinematics_map('r11'));
disp(kinematics_map('r12'));
disp(kinematics_map('r13'));
disp(kinematics_map('r21'));
disp(kinematics_map('r22'));
disp(kinematics_map('r23'));
disp(kinematics_map('r31'));
disp(kinematics_map('r32'));
disp(kinematics_map('r33'));
disp(kinematics_map('px'));
disp(kinematics_map('py'));
disp(kinematics_map('pz'));

%% Numerical Values 

[Num_T_final, Num_euler_ZYX, Num_fk_map,dh_table] = Num_calculate_robot_kinematics();

% --- DISPLAY RESULTS ---
fprintf('==============================================================================================================================================================================\n');
fprintf('Numerical Values transformation Matrix:\n');
fprintf('==================================================\n');
disp(Num_T_final);

fprintf('==================================================\n');
fprintf('ZYX Euler Angles (Yaw, Pitch, Roll):\n');
fprintf('==================================================\n');
disp(Num_euler_ZYX);

fprintf('==================================================\n');
fprintf('Translation:\n');
fprintf('==================================================\n');

Translation = [Num_fk_map('px'); Num_fk_map('py'); Num_fk_map('pz')];
disp(Translation);



%% Plot The Robot
Rob = SerialLink(dh_table, 'name', 'RRRRR')
q = dh_table(:, 1)';
Rob.plot(q, 'workspace', [-400 400 -400 400 -1 500]);
