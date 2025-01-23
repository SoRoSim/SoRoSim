function [con1, dcon1] = con1(Linkage, x)
q = x(1:Linkage.ndof);
u = x(Linkage.ndof+1:Linkage.ndof+12);
l = x(Linkage.ndof+13:Linkage.ndof+18);

[res, jac_q, jac_u, jac_l] = StaticResidueJacobian(Linkage, q, u, l);
con1 = res;
dcon1 = [jac_q, jac_u, jac_l];
end