function T_total = manual_dh_matrix(dh_table)
% manual_dh_matrix Calculates the total transformation matrix from a DH table.
%
% INPUT:
%   dh_table - An Nx4 symbolic matrix where each row is [theta, d, a, alpha].
%
% OUTPUT:
%   T_total  - The 4x4 symbolic total transformation matrix (T_0_N).
%   Additionally, individual transformation matrices are saved as A1, A2, ..., AN in the base workspace.

    % Get the number of links (rows in the table)
    num_links = size(dh_table, 1);
    Array_of_All_Matrix = cell(1, num_links);
    
    % Loop through each link (each row of the DH table)
    for i = 1:num_links
        % Extract parameters for the current link
        theta = dh_table(i, 1);
        d     = dh_table(i, 2);
        a     = dh_table(i, 3);
        alpha = dh_table(i, 4);
        
        % Calculate the transformation matrix for this single link
        trans = [
            cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,           sin(alpha),             cos(alpha),            d;
            0,           0,                      0,                     1
        ];
        
        % Store the transformation matrix for this link
        Array_of_All_Matrix{i} = trans;
    end

    % Compute the total transformation matrix
    T_total = Array_of_All_Matrix{1};
    for i = 2:num_links
        T_total = T_total * Array_of_All_Matrix{i};
    end
    
    % Save individual transformation matrices as A1, A2, ..., AN in the base workspace
    for i = 1:num_links
        assignin('base', ['A' num2str(i)], Array_of_All_Matrix{i});
    end
end