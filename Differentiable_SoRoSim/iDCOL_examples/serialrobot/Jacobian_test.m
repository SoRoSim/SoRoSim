q = randomConfiguration(robot);
endEffector = 'upper_arm_link';
J_space = geometricJacobian(robot, q, endEffector);
T = getTransform(robot, q, endEffector);

% J_body = adjointSE3_omega_v(ginv(T)) * J_space;
R = T(1:3,1:3);

J_body = [R.' * J_space(1:3,:);
          R.' * J_space(4:6,:)];
J_body_num = numericalBodyJacobian(robot, q, endEffector);

disp(norm(J_body - J_body_num))


function J_body_num = numericalBodyJacobian(robot, q, endEffector)
    h = 1e-6;

    % q = q(:).';
    n = numel(q);

    T0 = getTransform(robot, q, endEffector);

    J_body_num = zeros(6,n);

    for i = 1:n
        q_pert = q;
        q_pert(i) = q_pert(i) + h;

        T1 = getTransform(robot, q_pert, endEffector);

        % Body-frame relative motion
        T_rel = T0 \ T1;   % same as inv(T0)*T1

        % se(3) logarithm
        Xi_hat = logm(T_rel) / h;
        
        % Extract twist in [omega; v] order
        omega_hat = Xi_hat(1:3,1:3);
        v = Xi_hat(1:3,4);

        omega = [omega_hat(3,2);
                 omega_hat(1,3);
                 omega_hat(2,1)];

        J_body_num(:,i) = [omega; v];
    end
end
function AdT = adjointSE3_omega_v(T)
    R = T(1:3,1:3);
    p = T(1:3,4);

    p_hat = [  0    -p(3)  p(2);
              p(3)   0   -p(1);
             -p(2)  p(1)   0 ];

    AdT = [R, zeros(3,3);
           p_hat*R, R];
end