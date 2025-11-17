% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures
% Links Parameters

syms theta_01_rad theta_02_rad theta_03_rad theta_04_rad theta_05_rad theta_06_rad  % Joint variables
syms a1 a2 a3 d1 d2 d3                                                              % Link parameters
syms dummy
%a1 = 37.2;a2 = 138.10;a3 = 28.2;d1 = 135.80;d2 = 160;d3 = 15;



L=[theta_01_rad      d1    a1        pi/2 ;
 theta_02_rad      0      a2       -pi ;
  theta_03_rad      0      a3       -pi/2;
  theta_04_rad      d2     0        pi/2 ;
  theta_05_rad      0      0        -pi/2;
  theta_06_rad      d3     0        0];

Transformation_Matrix = manual_dh_matrix(L);
disp(Transformation_Matrix);

Transformation_Matrix_simple=simplify(Transformation_Matrix);

myDict = dictionary(["r11", "r12", "r13","px", "r21", "r22", "r23","py","r31", "r32", "r33","pz"], ...
                    [dummy, dummy, dummy,dummy, dummy, dummy,dummy, dummy, dummy,dummy, dummy, dummy]);

%myDict = dictionary([   "r11", "r12", "r13","px", ...
%                        "r21", "r22", "r23","py", ...
%                        "r31", "r32", "r33","pz"], ...
%                    [0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0]);

key_list = keys(myDict);
i = 1;

for x=1:3
    for y=1:4
        myDict(key_list(i)) = Transformation_Matrix_simple(x,y);
        i = i + 1;
    end
end


disp(myDict('pz'));


yaw = atan2(myDict('r23'), myDict('r13'));
pitch = atan2(sqrt(1 - myDict('r33')^2), myDict('r33'));
roll = atan2(myDict('r32'), -myDict('r31'));
