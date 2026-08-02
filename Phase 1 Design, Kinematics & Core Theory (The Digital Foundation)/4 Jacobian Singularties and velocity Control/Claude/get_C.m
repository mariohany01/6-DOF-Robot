function C = get_C(q_vec, dq_vec)
% Numerical Coriolis matrix via finite difference on M(q)
% q_vec  = [q1..q6]  current joint angles
% dq_vec = [dq1..dq6] current joint velocities

    n   = 6;
    C   = zeros(n,n);
    dh  = 1e-7;   % perturbation step

    % M at current q
    M0  = get_M(q_vec(1),q_vec(2),q_vec(3), ...
                q_vec(4),q_vec(5),q_vec(6));

    % dM/dqi for each joint i — numerical
    dMdq = zeros(n,n,n);
    for i = 1:n
        q_p    = q_vec;
        q_p(i) = q_p(i) + dh;
        M_p    = get_M(q_p(1),q_p(2),q_p(3), ...
                       q_p(4),q_p(5),q_p(6));
        dMdq(:,:,i) = (M_p - M0) / dh;
    end

    % Christoffel symbols
    for k = 1:n
        for j = 1:n
            c = 0;
            for i = 1:n
                c = c + 0.5*(dMdq(k,j,i) + ...
                             dMdq(k,i,j) - ...
                             dMdq(i,j,k)) * dq_vec(i);
            end
            C(k,j) = c;
        end
    end
end