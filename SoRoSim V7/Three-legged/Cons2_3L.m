function [c, ceq, dc, dceq] = Cons2_3L(Linkage, x, hole_position,leg_index, radius)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+Linkage.nact);
    l = x(Linkage.ndof+Linkage.nact+1:Linkage.ndof+Linkage.nact+Linkage.CLprecompute.nCLp);
    
    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = res;
    dceq = [jac_q, jac_u, jac_l]';
    dceq = [dceq; zeros(length(leg_index),Linkage.ndof+Linkage.CLprecompute.nCLp)];
    c = [];

    % %% Inequality constraints 
    dc = zeros(length(x), length(c));

    for i =1:length(leg_index)
        xbar1 = x(Linkage.ndof+Linkage.nact+Linkage.CLprecompute.nCLp+i);
        [g_xbar1, xi_xbar1, J_xbar1] = Screw_interpolate(Linkage,xbar1, q, leg_index(i));
        xh1 = g_xbar1(1:3,4);
        eq1 = norm(hole_position(i,:) - xh1)^2;
        c = [c; eq1 - 0.2*(radius - 0.01)^2];

        dc(Linkage.ndof+Linkage.nact+Linkage.CLprecompute.nCLp+i,1) = -2*(hole_position(i,:)' - xh1)'*g_xbar1(1:3,1:3)*xi_xbar1(4:6);
        dc(1:Linkage.ndof,i) = -2*(hole_position(i,:)' - xh1)'*g_xbar1(1:3,1:3)*J_xbar1(4:6,:);
    end
end