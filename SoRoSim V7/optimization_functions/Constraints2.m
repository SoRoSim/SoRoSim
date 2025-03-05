function [c, ceq, dc, dceq] = Constraints2(Linkage, x, hole_position, radius)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+12);
    l = x(Linkage.ndof+13:Linkage.ndof+18);
    
    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = res;
    dceq = [jac_q, jac_u, jac_l]';
    dceq = [dceq; zeros(2,Linkage.ndof+6*Linkage.nCLj)];
    c = [];

    % %% Inequality constraints 
    xbar1 = x(end-1);
    [g_xbar1, xi_xbar1, J_xbar1] = Screw_interpolate(Linkage,xbar1, q, 1);
    xh1 = g_xbar1(1:3,4);

    eq1 = norm(hole_position(1,:)'  - xh1)^2;


    c = [c; eq1 - 0.2*(radius - 0.01)^2];


    dc = zeros(length(x), length(c));
    dc(end-1,1) = -2*(hole_position(1,:)' - xh1)'*g_xbar1(1:3,1:3)*xi_xbar1(4:6);
    dc(1:Linkage.ndof,1) = -2*(hole_position(1,:)' - xh1)'*g_xbar1(1:3,1:3)*J_xbar1(4:6,:);
    xbar2 = x(end);
    [g_xbar2, xi_xbar2, J_xbar2] = Screw_interpolate(Linkage,xbar2, q, 3);
    xh2 = g_xbar2(1:3,4);

    eq2 = norm(hole_position(2,:)' - xh2)^2;


    c = [c; eq2 - 0.2*(radius - 0.01)^2];
 


    dc(end,2) = -2*(hole_position(2,:)' - xh2)'*g_xbar2(1:3,1:3)*xi_xbar2(4:6);
    dc(1:Linkage.ndof,2) = -2*(hole_position(2,:)' - xh2)'*g_xbar2(1:3,1:3)*J_xbar2(4:6,:);

end