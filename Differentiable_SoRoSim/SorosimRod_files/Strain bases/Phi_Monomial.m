function Phi = Phi_Monomial(X,Phi_dof,Phi_odr)
dof = sum(Phi_dof.*(Phi_odr+1));
Phi   = zeros(6,dof);

k = 1;
for i=1:6
    for j=1:Phi_dof(i)*(Phi_odr(i)+1)
        Phi(i,k) = X^(j-1);
        k = k+1;
    end
end

end