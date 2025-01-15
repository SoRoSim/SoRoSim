function Phi = Phi_Gaussian(X,Phi_dof,Phi_odr)

% X varies from 0 to 1
% Phi_dof tells which deformation modes are enabled
% eg. Phi_dof = [1 1 1 0 0 0]' implies all 3 rotational modes are enabled
% Phi_odr defines the order of each strains
% eg. Phi_odr = [0 1 1 0 0 0]' implies linear bending strains while other are
% constant strains

dof = sum(Phi_dof.*(Phi_odr+1));
Phi   = zeros(6,dof);

k = 1;
for i=1:6
    if Phi_odr(i)==0&&Phi_dof(i)==1
        Phi(i,k) = 1;
        k = k+1;
    else
        w = 1/Phi_odr(i);
        a = 0;
        c = 2*sqrt(log(2))/w; % to have a value of 0.5 at w/2
        for j=1:Phi_dof(i)*(Phi_odr(i)+1)
            Phi(i,k) = exp(-(X-a)^2*c^2);
            k = k+1;
            a = a+w;
        end
    end
end

end