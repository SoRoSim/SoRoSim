function [c, ceq, dc, dceq] = Constraint3(Linkage, x, n_points, g_des_initial)
    c = [];
    ceq = [];
    dc = [];
    dceq= [];
    num_variables= length(x)/n_points;
    for i = 1:n_points
        q = x((i-1)*num_variables + 1:(i-1)*num_variables +Linkage.ndof);
        u = x((i-1)*num_variables + 1 + Linkage.ndof:(i-1)*num_variables + Linkage.ndof + 12);
        l = x((i-1)*num_variables + Linkage.ndof + 13: (i-1)*num_variables + Linkage.ndof + 18);
        
        [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
        ceq = [ceq; res];
        dceq = [dceq; jac_q'; jac_u'; jac_l'];
    end
    q_in = x(1:Linkage.ndof);
    g = Linkage.FwdKinematics(q_in,2);
    g_plat = g(5:8,1:4);
    e2 = piecewise_logmap(ginv(g_des_initial)*g_plat);
    ceq = [ceq; e2];
    J_platform = Linkage.Jacobian(q_in, 2);
    J_platform = J_platform(7:12,:);
    [~,Teq2] = variable_expmap_gTg(e2);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des_initial)*g_plat);
    Je2 = zeros(780,6);
    Je2(1:Linkage.ndof,:) = (Teq2\(Ad_exp_eq2*J_platform))';


    dceq = [dceq Je2];
end