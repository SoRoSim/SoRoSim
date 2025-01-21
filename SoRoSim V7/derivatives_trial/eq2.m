function [eq2, deq2] = eq2(S1, qu_uq_l, g_des_final)
    qT = qu_uq_l(end, 1:S1.ndof);
    gT = S1.FwdKinematics(qT);
    g_T = gT(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    eq2 = piecewise_logmap(ginv(g_des_final)*g_T);
    % eq2 = norm(e2)^2;

    deq2 = zeros(6,80);
    [~,Teq2] = variable_expmap_gTg(eq2);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des_final)*g_T);
    J_platform = S1.Jacobian(qu_uq_l(1:S1.ndof), 2);
    J_platform = J_platform(7:12,:);
    deq2(:,1:60) = Teq2\(Ad_exp_eq2*J_platform);
end
    
