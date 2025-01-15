function Phi = Phi_Chebychev(X,Phi_dof,Phi_odr)

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
    T0 = 1;
    T1 = X;
    for j=1:Phi_dof(i)*(Phi_odr(i)+1)
        if j==1
            Phi(i,k) = T0;
        end
        if j==2
           Phi(i,k) = T1; 
        end
        if j>=3
            T1t = T1;
            T1 = 2*X*T1-T0;
            T0 = T1t;
            Phi(i,k) = T1;
        end
        k = k+1;
    end
end

end