
function [c, ceq, dc, dceq] = Constraints4(Linkage, x, n_points, g_des_initial, g_des_final, constraint_surface)    
    c = [];
    ceq = [];
    num_variables= length(x)/n_points;
    pose_constraints = 1;
    n_equations = (Linkage.ndof + Linkage.nCLj * 6)*n_points;
    n_inequalities = 2*n_points;
    dc = zeros(length(x), n_inequalities);
    dceq = zeros(length(x), n_equations);
    for i = 1:n_points
        q = x((i-1)*num_variables + 1:(i-1)*num_variables +Linkage.ndof);
        u = x((i-1)*num_variables + 1 + Linkage.ndof:(i-1)*num_variables + Linkage.ndof + 12);
        l = x((i-1)*num_variables + Linkage.ndof + 13: (i-1)*num_variables + Linkage.ndof + 18);
        
        [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
        ceq = [ceq; res];

        dceq((i-1)*num_variables +1:i*num_variables-2, (i-1)*66+1:i*66) = [jac_q jac_u jac_l]';  %% Remove the 66 and make it general later


        xbar1 = x((i-1)*num_variables + Linkage.ndof + 19);
        [g_xbar1, xi_xbar1, J_xbar1] = Screw_interpolate(Linkage,xbar1, q, 1);
        xh1 = g_xbar1(1:3,4);

        eq1 = norm(constraint_surface.hole_1  - xh1)^2;

        c = [c; eq1 - 0.2*(constraint_surface.radius + 0.01)^2];
        % i
        dc((i-1)*num_variables + Linkage.ndof + 19,2*i-1) = -2*(constraint_surface.hole_1 - xh1)'*g_xbar1(1:3,1:3)*xi_xbar1(4:6);
        dc((i-1)*num_variables + 1:(i-1)*num_variables +Linkage.ndof,2*1-1) = -2*(constraint_surface.hole_1 - xh1)'*g_xbar1(1:3,1:3)*J_xbar1(4:6,:);
        
        xbar2 = x((i-1)*num_variables + Linkage.ndof + 20);
        [g_xbar2, xi_xbar2, J_xbar2] = Screw_interpolate(Linkage,xbar2, q, 3);
        xh2 = g_xbar2(1:3,4);
    
        eq2 = norm(constraint_surface.hole_2 - xh2)^2;
    
        c = [c; eq2 - 0.2*(constraint_surface.radius + 0.01)^2];
        % g_xbar2
        % J_xbar2
        dc((i-1)*num_variables + Linkage.ndof + 20,2*i) = -2*(constraint_surface.hole_2 - xh2)'*g_xbar2(1:3,1:3)*xi_xbar2(4:6);
        dc((i-1)*num_variables + 1:(i-1)*num_variables +Linkage.ndof,2*i) = -2*(constraint_surface.hole_2 - xh2)'*g_xbar2(1:3,1:3)*J_xbar2(4:6,:);

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
    Je2 = zeros(length(x), 6);
    Je2(1:Linkage.ndof,1:6) = (Teq2\(Ad_exp_eq2*J_platform))';
    qT = x(9*num_variables + 1:9*num_variables +Linkage.ndof);
    gT = Linkage.FwdKinematics(qT);
    g_T = gT(4*(length(Linkage.CVRods{1}(2).Xs)+2)+1:4*(length(Linkage.CVRods{1}(2).Xs)+3),:);
    e3 = piecewise_logmap(ginv(g_des_final)*g_T);
    ceq = [ceq; e3];
    [~,Teq2] = variable_expmap_gTg(e2);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des_final)*g_T);
    J_platform = Linkage.Jacobian(x(1:Linkage.ndof), 2);
    J_platform = J_platform(7:12,:);
    Je3 = zeros(length(x), 6);
    Je3(9*num_variables + 1:9*num_variables +Linkage.ndof,1:6) = (Teq2\(Ad_exp_eq2*J_platform))';


    dceq = [dceq Je2 Je3];