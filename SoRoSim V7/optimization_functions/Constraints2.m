function [c, ceq, dc, dceq] = Constraints2(Linkage, x, hole_position, radius,normal)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+12);
    l = x(Linkage.ndof+13:Linkage.ndof+18);
    xbar1 = x(end-1);
    [g_xbar1, xi_xbar1, J_xbar1] = Screw_interpolate(Linkage,xbar1, q, 1);
    dpdq1 = g_xbar1(1:3,1:3)*J_xbar1(4:6,:);
    dpdx1 = g_xbar1(1:3,1:3)*xi_xbar1(4:6)*Linkage.VLinks(1).ld{1};
    xbar2 = x(end);
    [g_xbar2, xi_xbar2, J_xbar2] = Screw_interpolate(Linkage,xbar2, q, 3);
    dpdq2 = g_xbar2(1:3,1:3)*J_xbar2(4:6,:);
    dpdx2 = g_xbar2(1:3,1:3)*xi_xbar2(4:6)*Linkage.VLinks(1).ld{1};
    
    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = [res;g_xbar1(3,4) - hole_position(1,3); g_xbar2(3,4) - hole_position(1,3)] ;
    dceq = [jac_q, jac_u, jac_l]';
    dceq = [dceq; zeros(2,Linkage.ndof+6*Linkage.nCLj)];
    dceq = [dceq, [dpdq1(3,:)'; zeros(Linkage.nact,1); zeros(Linkage.nCLj*6,1); dpdx1(3);0]];
    dceq = [dceq, [dpdq2(3,:)'; zeros(Linkage.nact,1); zeros(Linkage.nCLj*6,1); 0;dpdx2(3)]];

    
    % %% Inequality constraints 
    c = [];
    xh1 = g_xbar1(1:3,4);
    eq1 = norm(hole_position(1,1:2)'  - xh1(1:2))^2;
    c = [c; eq1 - 0.2*(radius - 0.01)^2];
    
    xh2 = g_xbar2(1:3,4);
    eq2 = norm(hole_position(2,1:2)' - xh2(1:2))^2;
    c = [c; eq2 - 0.2*(radius - 0.01)^2];
    % Add the directional constraints
    x_hat1 = g_xbar1(1:3,1);
    n1 = normal(:,1);
    c = [c; -x_hat1'*n1];
    x_hat2 = g_xbar2(1:3,1);
    n2 = normal(:,2);
    c = [c; -x_hat2'*n2];

    dc = zeros(length(x), length(c));
    dc(end-1,1) = -2*(hole_position(1,1:2)' - xh1(1:2))'*dpdx1(1:2);
    dc(1:Linkage.ndof,1) = -2*(hole_position(1,1:2)' - xh1(1:2))'*dpdq1(1:2,:);
    dc(end,2) = -2*(hole_position(2,1:2)' - xh2(1:2))'*dpdx2(1:2);
    dc(1:Linkage.ndof,2) = -2*(hole_position(2,1:2)' - xh2(1:2))'*dpdq2(1:2,:);
    
    dc(end-1,3) = R*xi_xbar2 
    dc(1:Linkage.ndof,3) = g_xbar1(1:3,1:3)*J_xbar1(1:3,:);


end