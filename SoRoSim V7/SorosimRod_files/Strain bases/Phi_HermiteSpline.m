function Phi = Phi_HermiteSpline(X,Phi_dof,Phi_odr)

% X varies from 0 to 1
% Phi_dof tells which deformation modes are enabled
% eg. Phi_dof = [1 1 1 0 0 0]' implies all 3 rotational modes are enabled

nele = max(Phi_odr); % Gives the number of elements
dof = sum(Phi_dof*(2*nele+2));
Phi   = zeros(6,dof);

k = 1;
for i=1:6
    w = 1/nele;
    a = 0;
        
    for j=1:Phi_dof(i)*nele
        b = a+w;
        if X>=a&&X<=b
            Xc = (X-a)/(b-a);
            Phi(i,k)   = 1-3*Xc^2+2*Xc^3;
            Phi(i,k+1) = Xc-2*Xc^2+Xc^3;
            Phi(i,k+2) = 3*Xc^2-2*Xc^3;
            Phi(i,k+3) = -Xc^2+Xc^3;
        end
        if j==nele
            k = k+3;
        else
            k = k+2;
        end
        a = a+w;
    end
        
    k = k+Phi_dof(i);
end

end