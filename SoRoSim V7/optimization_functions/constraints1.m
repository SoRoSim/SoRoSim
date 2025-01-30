function [c,ceq,dc, dceq] = constraints1(Linkage, x)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+12);
    l = x(Linkage.ndof+13:Linkage.ndof+18);
    
    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = res;
    dceq = [jac_q, jac_u, jac_l]'; %Transpose for fmincon
    c = [];
    dc = [];

end