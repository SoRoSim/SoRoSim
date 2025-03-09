function [c,ceq,dc, dceq] = cons1_3L(Linkage, x)
    q = x(1:Linkage.ndof);
    u = x(Linkage.ndof+1:Linkage.ndof+18);
    l = x(Linkage.ndof+19:Linkage.ndof+30);
    
    [res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
    ceq = res;
    dceq = [jac_q, jac_u, jac_l]'; %Transpose for fmincon
    c = [];
    dc = [];

end