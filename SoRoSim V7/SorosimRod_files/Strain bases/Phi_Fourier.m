function Phi = Phi_Fourier(X,Phi_dof,Phi_odr)
dof = sum(Phi_dof.*(2*Phi_odr+1));
Phi   = zeros(6,dof);

k = 1;
for i=1:6
    for j=1:Phi_dof(i)*(Phi_odr(i)+1)
        if j==1
            Phi(i,k) = 1;
            k = k+1;
        else
            Phi(i,k)   = cos(2*pi*(j-1)*X);
            Phi(i,k+1) = sin(2*pi*(j-1)*X);
            k = k+2;
        end
    end
end

end