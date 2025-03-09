function [c, ceq, dc, dceq] = Constraints2(Linkage, x, hole_position, radius)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+12);
    l = x(Linkage.ndof+13:Linkage.ndof+18);
    xbar1 = x(end-1);
    xbar2 = x(end);
    [g_xbar1, xi_xbar1, J_xbar1] = Screw_interpolate(Linkage,xbar1, q, 1);
    [g_xbar2, xi_xbar2, J_xbar2] = Screw_interpolate(Linkage,xbar2, q, 3);
    xh2 = g_xbar2(1:3,4);
    xh1 = g_xbar1(1:3,4);
    Rxi1 = g_xbar1(1:3,1:3)*xi_xbar1(4:6);
    RJ1 = g_xbar1(1:3,1:3)*J_xbar1(4:6,:);
    Rxi2 = g_xbar2(1:3,1:3)*xi_xbar2(4:6);
    RJ2 = g_xbar2(1:3,1:3)*J_xbar2(4:6,:);

    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = [res; (hole_position(1,3) - xh1(3)); (hole_position(2,3) - xh2(3))];

    dceq = [jac_q, jac_u, jac_l]';
    dceq = [dceq; zeros(2,Linkage.ndof+6*Linkage.nCLj)];
    dceq = [dceq, [RJ1(3,:)'; zeros(Linkage.nact+6*Linkage.nCLj,1);Rxi1(3);0]];
    dceq = [dceq, [RJ2(3,:)'; zeros(Linkage.nact+6*Linkage.nCLj,1);0; Rxi2(3)]];
    c = [];

    % %% Inequality constraints 
    
    xh1 = g_xbar1(1:3,4);

    eq1 = norm(hole_position(1,1:2)'  - xh1(1:2))^2;


    c = [c; eq1 - 0.2*(radius - Linkage.VLinks(1).r{1}(0))^2];


    dc = zeros(length(x), length(c));
    
    dc(end-1,1) = -2*(hole_position(1,1:2)' - xh1(1:2))'*Rxi1(1:2);
    dc(1:Linkage.ndof,1) = -2*(hole_position(1,1:2)' - xh1(1:2))'*RJ1(1:2,:);

    eq2 = norm(hole_position(2,1:2)' - xh2(1:2))^2;


    c = [c; eq2 - 0.2*(radius - Linkage.VLinks(3).r{1}(0))^2];
 

    
    dc(end,2) = -2*(hole_position(2,1:2)' - xh2(1:2))'*Rxi2(1:2);
    dc(1:Linkage.ndof,2) = -2*(hole_position(2,1:2)' - xh2(1:2))'*RJ2(1:2,:);
end