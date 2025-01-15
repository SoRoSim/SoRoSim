function dSTdq_FC = compute_dSTdqFC_Z2R(ndof,Omega,Z,f,fd,adjOmegap,F_C)

dSTdq_FC = zeros(ndof,ndof);
theta = norm(Omega(1:3));
coadjOmegap = zeros(24,6);

if (theta<=1e-2)
    
    for r=1:4
        coadjOmegap((r-1)*6+1:6*r,:) = (-1)^r*adjOmegap((r-1)*6+1:6*r,:)'; % good because u is from 1 to r
        for u=1:r
            if u==1
                coadjOmegap1 = eye(6);
            else
                coadjOmegap1 = coadjOmegap((u-2)*6+1:6*(u-1),:); %coadjOmega^(u-1)
            end
            if u==r
                coadjOmegap2 = eye(6);
            else
                coadjOmegap2 = coadjOmegap((r-u-1)*6+1:6*(r-u),:); %coadjOmega^(r-u)
            end
            dSTdq_FC = dSTdq_FC+(-1)^r*f(r)*Z'*coadjOmegap1*dinamico_coadjbar(coadjOmegap2*F_C)*Z;
        end
    end

else

    I_theta = diag([1 1 1 0 0 0]);
    for r=1:4
        dSTdq_FC = dSTdq_FC+1/theta*fd(r)*Z'*adjOmegap((r-1)*6+1:6*r,:)'*F_C*Omega'*I_theta*Z; %(-1)^r*coadjOmegap = adjOmegap 
        coadjOmegap((r-1)*6+1:6*r,:) = (-1)^r*adjOmegap((r-1)*6+1:6*r,:)'; % good because u is from 1 to r
        for u=1:r
            if u==1
                coadjOmegap1 = eye(6);
            else
                coadjOmegap1 = coadjOmegap((u-2)*6+1:6*(u-1),:); %coadjOmega^(u-1)
            end
            if u==r
                coadjOmegap2 = eye(6);
            else
                coadjOmegap2 = coadjOmegap((r-u-1)*6+1:6*(r-u),:); %coadjOmega^(r-u)
            end
            dSTdq_FC = dSTdq_FC+(-1)^r*f(r)*Z'*coadjOmegap1*dinamico_coadjbar(coadjOmegap2*F_C)*Z;
        end
    end
    
end

end