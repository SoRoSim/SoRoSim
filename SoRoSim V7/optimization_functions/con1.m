function [c, ceq,dc, dceq] = con1(Linkage, x, constraint_surface)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+12);
    l = x(Linkage.ndof+13:Linkage.ndof+18);
    
    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = res;
    dceq = [jac_q, jac_u, jac_l]';
    c =[];
    %% Inequality constraints
    xbar1 = x(79);
    Xs = Linkage.CVRods{1}(2).Xs;
    g = Linkage.FwdKinematics(x(1:Linkage.ndof));
    %Screw interpolation
    index = find(Xs>=xbar1,1);
    alpha = (xbar1 - Xs(index-1))/(Xs(index)- Xs(index - 1));
    xi = piecewise_logmap(ginv(g(4*(index-1) - 3:4*(index-1),:))*g(4*index-3:4*index,:));
    
    g_xbar1 = variable_expmap_gTg(xi*alpha);
    xh1 = g_xbar1(1:3,4);

    eq1 = norm([constraint_surface.hole_1 constraint_surface.height] - xh1)^2;
    c = [c; eq1 - (constraint_surface.radius + 0.01)^2];
    xi = Linkage.ScrewStrain(x(1:Linkage.ndof),1);
    xi_index = xi(6*index + 1: 6*(index+1));
    xi_pre_index = xi(6*(index-1)+1:6*index);
    xi_xbar1 = (1-alpha)*xi_pre_index + alpha * xi_index;

    xbar1 = x(80);
    Xs = Linkage.CVRods{1}(2).Xs;
    g = Linkage.FwdKinematics(x(1:Linkage.ndof));
    index = find(Xs>=xbar1,1);
    alpha = (xbar1 - Xs(index-1))/(Xs(index)- Xs(index - 1));
    xi = piecewise_logmap(ginv(g(4*(index-1) - 3:4*(index-1),:))*g(4*index-3:4*index,:));
    g_xbar2 = variable_expmap_gTg(xi*alpha);
    xh2 = g_xbar2(1:3,4);

    eq1 = norm([constraint_surface.hole_2 constraint_surface.height] - xh2)^2;
    c = [c; eq1 - (constraint_surface.radius + 0.01)^2];
    xi = Linkage.ScrewStrain(x(1:Linkage.ndof),1);
    xi_index = xi(6*index + 1: 6*(index+1));
    xi_pre_index = xi(6*(index-1)+1:6*index);
    xi_xbar2 = (1-alpha)*xi_pre_index + alpha * xi_index;

    dc = zeros(length(x), length(c));

    dc(79,1) = -2*([constraint_surface.hole_1 constraint_surface.height]' - xh1)'*g_xbar1(1:3,1:3)*xi_xbar1(4:6);
    dc(80,2) = -2*([constraint_surface.hole_2 constraint_surface.height]' - xh2)'*g_xbar2(1:3,1:3)*xi_xbar2(4:6);

end