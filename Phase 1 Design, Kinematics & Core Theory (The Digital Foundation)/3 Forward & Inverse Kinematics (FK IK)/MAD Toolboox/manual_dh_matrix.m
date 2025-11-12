function Transformation_Matrix = manual_dh_matrix(dh_parameters)
    
    num_links = size(dh_parameters, 1);
    Array_of_All_Matrix = cell(1,num_links);

    for i= 1:num_links
        theta = dh_parameters(i,1);
        d = dh_parameters(i, 2);
        a = dh_parameters(i, 3);
        alpha = dh_parameters(i,4);

        trans = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                0,          sin(alpha),             cos(alpha),             d;
                0,          0,                      0,                      1];
        
        Array_of_All_Matrix{i} = trans;
    end

    Transformation_Matrix = Array_of_All_Matrix{1};

    for i = 2:num_links
        Transformation_Matrix = Transformation_Matrix * Array_of_All_Matrix{i};
    end
    
end