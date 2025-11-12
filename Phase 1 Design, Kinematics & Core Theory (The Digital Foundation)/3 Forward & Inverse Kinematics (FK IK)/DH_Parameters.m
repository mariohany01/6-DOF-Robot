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

%%
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
Rob.plot(q, 'workspace', [-400 400 -400 400 -1 500]);
T = Rob.fkine(q);

% Eliminate very small values from T (set values below threshold to 0)
threshold = 1e-10;
T_matrix = double(T);  % Convert to double matrix if needed
T_matrix(abs(T_matrix) < threshold) = 0;
disp(T_matrix);

% Sliders
% Rob.teach;