function [p,d,dp,dd] = position_derivatives(S1, x)
    d = [];
    dd = [];
    q = x(1:S1.ndof);
    u = x(S1.ndof+1:S1.ndof+S1.nact);
    l = x(S1.ndof+S1.nact+1: S1.ndof+S1.nact+S1.nCLj*6);
    xbar1 = x(S1.ndof+S1.nact+S1.nCLj*6+1);


    [g_xbar1, xi_xbar1, J_xbar1] = Screw_interpolate(S1,xbar1, q, 1);
    p = g_xbar1(1:3,4);

    dpdq = g_xbar1(1:3,1:3)*J_xbar1(4:6,:);
    dpdu = zeros(3,S1.nact);
    dpdl = zeros(3,6*S1.nCLj);
    dpdxbar = g_xbar1(1:3,1:3)*xi_xbar1(4:6);

    dp = [dpdq'; dpdu'; dpdl'; dpdxbar'];

end