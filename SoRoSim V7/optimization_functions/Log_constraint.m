function [c, dc] = Log_constraint(Linkage, x, g_des)
    g = Linkage.FwdKinematics(x,2);
    g_plat = g(5:8,1:4);
    c = piecewise_logmap(ginv(g_des)*g_plat);
    J_platform = Linkage.Jacobian(x, 2);
    J_platform = J_platform(7:12,:);
    [~,Teq2] = variable_expmap_gTg(c);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des)*g_plat);
    dc = zeros(length(x), 6);
    dc(1:Linkage.ndof,1:6) = (Teq2\(Ad_exp_eq2*J_platform))';
end