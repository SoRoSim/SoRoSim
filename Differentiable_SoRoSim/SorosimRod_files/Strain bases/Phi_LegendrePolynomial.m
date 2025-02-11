function Phi = Phi_LegendrePolynomial(X,Phi_dof,Phi_odr)

% X varies from 0 to 1
% Phi_dof tells which deformation modes are enabled
% eg. Phi_dof = [1 1 1 0 0 0]' implies all 3 rotational modes are enabled
% Phi_odr defines the order of each strains
% eg. Phi_odr = [0 1 1 0 0 0]' implies linear bending strains while other are
% constant strains

dof = sum(Phi_dof.*(Phi_odr+1));
Phi   = zeros(6,dof);

k = 1;
X = 2*X-1; %transform from [0,1] to [-1 1]
for i=1:6
    P0 = 1;
    P1 = X;
    for j=1:Phi_dof(i)*(Phi_odr(i)+1)
        if j==1
            Phi(i,k) = P0;
        end
        if j==2
           Phi(i,k) = P1; 
        end
        if j>=3
            n=j-2;
            P1t = P1;
            P1 = ((2*n+1)*X*P1-n*P0)/(n+1);
            P0 = P1t;
            Phi(i,k) = P1;
        end
        k = k+1;
    end
end


end