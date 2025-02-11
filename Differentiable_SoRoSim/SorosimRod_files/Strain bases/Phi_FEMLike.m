function Phi = Phi_FEMLike(X,Phi_dof,Phi_odr,SubClass)

% X varies from 0 to 1
% Phi_dof tells which deformation modes are enabled
% eg. Phi_dof = [1 1 1 0 0 0]' implies all 3 rotational modes are enabled
% Linear, Quadratic and Cubic FEM basis

if isempty(SubClass) %Default
    SubClass = 'Linear';
end

nele = max(Phi_odr); % Gives the number of elements

if strcmp(SubClass,'Linear')
    dof = sum(Phi_dof*(nele+1));
elseif strcmp(SubClass,'Quadratic')
    dof = sum(Phi_dof*(2*nele+1));
else % cubic
    dof = sum(Phi_dof*(3*nele+1));
end
Phi   = zeros(6,dof);

k = 1;
for i=1:6
    w = 1/nele;
    a = 0;
    if strcmp(SubClass,'Linear')

        for j=1:Phi_dof(i)*nele
            b = a+w;
            if X>=a&&(X<b||(X-b)^2<1e-12)
                Xc = (X-a)/(b-a);
                Phi(i,k) = 1-Xc;
                Phi(i,k+1) = Xc;
            end
            k = k+1;
            a = a+w;
        end
        
    elseif strcmp(SubClass,'Quadratic')

        for j=1:Phi_dof(i)*nele
            b = a+w;
            if X>=a&&X<=b
                Xc = (X-a)/(b-a);
                Phi(i,k)   =  2*(Xc-1/2)*(Xc-1);
                Phi(i,k+1) = -4*Xc*(Xc-1);
                Phi(i,k+2) =  2*Xc*(Xc-1/2);
            end
            k = k+2;
            a = a+w;
        end
        
    else %Cubic
        
        for j=1:Phi_dof(i)*nele
            b = a+w;
            if X>=a&&X<=b
                Xc = (X-a)/(b-a);
                Phi(i,k)   = -9/2*(Xc-1/3)*(Xc-2/3)*(Xc-1);
                Phi(i,k+1) =  27/2*Xc*(Xc-2/3)*(Xc-1);
                Phi(i,k+2) = -27/2*Xc*(Xc-1/3)*(Xc-1);
                Phi(i,k+3) =  9/2*Xc*(Xc-1/3)*(Xc-2/3);
            end
            k = k+3;
            a = a+w;
        end
        
    end
    k = k+Phi_dof(i);
end

end