function [eq2, deq2] = eq2(S1, x, g_des)
%% This is correct
    qT = x;
    gT = S1.FwdKinematics(qT);
    g_T = gT(4*(length(S1.CVRods{1}(2).Xs)+2)+1:4*(length(S1.CVRods{1}(2).Xs)+3),:);
    e2 = piecewise_logmap(ginv(g_des)*g_T);
    eq2 = norm(e2)^2;

    deq2 = zeros(size(x));
    [~,Teq2] = variable_expmap_gTg(e2);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des)*g_T);
    J_platform = S1.Jacobian(qT, 2);
    J_platform = J_platform(7:12,:);
    deq2(1:length(x)) = 2*e2' * (Teq2\(Ad_exp_eq2*J_platform));
end
    
