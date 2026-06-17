function J = IK_Objective_function(Linkage, q_a, g_des)
    q = zeros(Linkage.ndof,1);
    q(1:7) = q_a(1:7);
    q(end-6:end) = q_a(8:14);
    g_left = Linkage.FwdKinematics(q,8);
    g_left = g_left(5:8,:);
    g_right = Linkage.FwdKinematics(q,19);
    g_right = g_right(5:8,:);
    J = norm(piecewise_logmap(ginv(g_des(1:4,:))*g_left)) + norm(piecewise_logmap(ginv(g_des(5:8,:))*g_right));
end
