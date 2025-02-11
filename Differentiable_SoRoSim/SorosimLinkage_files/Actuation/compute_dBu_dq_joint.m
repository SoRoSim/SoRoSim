function dBu_dq = compute_dBu_dq_joint(Omega,S,Phi,f,fd,adjOmegap,Fk0)

dBu_dq = S'*dinamico_coadjbar(Fk0)*S;
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
            dBu_dq = dBu_dq+(-1)^r*f(r)*Phi'*coadjOmegap1*dinamico_coadjbar(coadjOmegap2*Fk0)*Phi;
        end
    end

else

    I_theta = diag([1 1 1 0 0 0]);
    for r=1:4
        dBu_dq = dBu_dq+1/theta*fd(r)*Phi'*adjOmegap((r-1)*6+1:6*r,:)'*Fk0*Omega'*I_theta*Phi; %(-1)^r*coadjOmegap = adjOmegap 
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
            dBu_dq = dBu_dq+(-1)^r*f(r)*Phi'*coadjOmegap1*dinamico_coadjbar(coadjOmegap2*Fk0)*Phi;
        end
    end
    
end

end