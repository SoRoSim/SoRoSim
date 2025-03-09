function [J, dJ] = obj1_3L(Linkage, x, g_des)
    q = x(1:Linkage.ndof);
    g = Linkage.FwdKinematics(q,2);
    g_plat = g(5:8,1:4);
    e = piecewise_logmap(ginv(g_des)*g_plat);
    J = norm(e)^2;

    dJ = zeros(size(x));
    [~,Teq2] = variable_expmap_gTg(e);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des)*g_plat);
    J_platform = Linkage.Jacobian(q, 2);
    J_platform = J_platform(7:12,:);
    dJ(1:Linkage.ndof) = 2*e' * (Teq2\(Ad_exp_eq2*J_platform));
end