function [E, dE] = objective_function_instance(S1, x, g_desired)
    q = x;
    gs = S1.FwdKinematics(q);
    g_platform = gs(4*(length(S1.CVRods{1}(2).Xs)+2)+1:4*(length(S1.CVRods{1}(2).Xs)+3),:);
    e = piecewise_logmap(ginv(g_desired)*g_platform);
    E = norm(e);

    dE = zeros(1,length(x));
    [~, T_e] = variable_expmap_gTg(e);
    Ad_e = dinamico_Adjoint(ginv(g_desired)*g_platform);
    J = S1.Jacobian(q,2);
    
    dE(1:S1.ndof) = piecewise_logmap(ginv(g_desired)*g_platform)'*(T_e\Ad_e*J(7:12,:));
end