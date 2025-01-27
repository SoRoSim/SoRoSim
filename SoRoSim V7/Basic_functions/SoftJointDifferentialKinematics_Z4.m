function [Omega,Z,g,T,S,Sd,f,fd,adjOmegap,dSdq_qd,dSdq_qdd,dSddq_qd]=SoftJointDifferentialKinematics_Z4(h,Phi_Z1,Phi_Z2,xi_star_Z1,xi_star_Z2,q,qd,qdd)
    
    Zc      = sqrt(3)/12;
    I_theta = diag([1 1 1 0 0 0]);
    hp2     = h*h;
    Zchp2   = Zc*hp2;

    xi_Z1 = Phi_Z1*q+xi_star_Z1;
    xi_Z2 = Phi_Z2*q+xi_star_Z2;

    xid_Z1 = Phi_Z1*qd;
    xid_Z2 = Phi_Z2*qd;

    xidd_Z1 = Phi_Z1*qdd;
    xidd_Z2 = Phi_Z2*qdd;

    Omega   = h/2*(xi_Z1+xi_Z2)+Zchp2*dinamico_adj(xi_Z1)*xi_Z2;
    Z       = h/2*(Phi_Z1+Phi_Z2)+Zchp2*(dinamico_adj(xi_Z1)*Phi_Z2-dinamico_adj(xi_Z2)*Phi_Z1);
    Omegad  = Z*qd;
    Zd      = Zchp2*(dinamico_adj(xid_Z1)*Phi_Z2-dinamico_adj(xid_Z2)*Phi_Z1);
    Zdd     = Zchp2*(dinamico_adj(xidd_Z1)*Phi_Z2-dinamico_adj(xidd_Z2)*Phi_Z1);

    %% defining T, Td, f, fd, fdd, adjOmegap (powers of adjOmega)

    f   = zeros(4,1);
    fd  = zeros(4,1);
    fdd = zeros(4,1);

    adjOmegap  = zeros(24,6); %all powers of adjGamma || INCLUDE POWER OF 0 to avoid an if
    adjOmegapd = zeros(24,6); %derivative of adjGammap

    k      = Omega(1:3);
    theta  = norm(k);
    kd     = Omegad(1:3);
    thetad = (kd'*k)/theta;
    
    Omegahat  = dinamico_hat(Omega);
    Omegahatp2 = Omegahat*Omegahat;
    Omegahatp3 = Omegahatp2*Omegahat;

    adjOmegap(1:6,:)  = dinamico_adj(Omega);
    adjOmegap(7:12,:)  = adjOmegap(1:6,:)*adjOmegap(1:6,:);
    adjOmegap(13:18,:) = adjOmegap(7:12,:)*adjOmegap(1:6,:);
    adjOmegap(19:24,:) = adjOmegap(13:18,:)*adjOmegap(1:6,:);

    adjOmegapd(1:6,:) = dinamico_adj(Omegad);
    adjOmegapd(7:12,:)  = adjOmegapd(1:6,:)*adjOmegap(1:6,:)+adjOmegap(1:6,:)*adjOmegapd(1:6,:);
    adjOmegapd(13:18,:) = adjOmegapd(7:12,:)*adjOmegap(1:6,:)+adjOmegap(7:12,:)*adjOmegapd(1:6,:);
    adjOmegapd(19:24,:) = adjOmegapd(13:18,:)*adjOmegap(1:6,:)+adjOmegap(13:18,:)*adjOmegapd(1:6,:);

    if (theta<=1e-2)
        g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+Omegahatp2/2+Omegahatp3/6;

        f(1) = 1/2;
        f(2) = 1/6;
        f(3) = 1/24;
        f(4) = 1/120; %fd is already initialized to 0

        T  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+f(1)*adjOmegap(1:6,:)+f(2)*adjOmegap(7:12,:)+f(3)*adjOmegap(13:18,:)+f(4)*adjOmegap(19:24,:);
        Td = f(1)*adjOmegapd(1:6,:)+f(2)*adjOmegapd(7:12,:)+f(3)*adjOmegapd(13:18,:)+f(4)*adjOmegapd(19:24,:);
    else
        
        tp2        = theta*theta;
        tp3        = tp2*theta;
        tp4        = tp3*theta;
        tp5        = tp4*theta;
        tp6        = tp5*theta;
        tp7        = tp6*theta;
        
        sintheta   = sin(theta);
        costheta   = cos(theta);
        
        t1 = theta*sintheta;
        t2 = theta*costheta;
        t3 = -8+(8-tp2)*costheta+5*t1;
        t4 = -8*theta+(15-tp2)*sintheta-7*t2;
        t3d = 5*sintheta+sintheta*(tp2 - 8)+3*theta*costheta;
        t4d = -8+5*theta*sintheta+(15-tp2)*costheta-7*costheta;
    
        f(1) = (4-4*costheta-t1)/(2*tp2);
        f(2) = (4*theta-5*sintheta+t2)/(2*tp3);
        f(3) = (2-2*costheta-t1)/(2*tp4);
        f(4) = (2*theta-3*sintheta+t2)/(2*tp5);
    
        fd(1) = (t3)/(2*tp3);
        fd(2) = (t4)/(2*tp4);
        fd(3) = (t3)/(2*tp5);
        fd(4) = (t4)/(2*tp6);
    
        fdd(1) = (theta*t3d-3*t3)/(2*tp4);
        fdd(2) = (theta*t4d-4*t4)/(2*tp5);
        fdd(3) = (theta*t3d-5*t3)/(2*tp6);
        fdd(4) = (theta*t4d-6*t4)/(2*tp7);
        
        g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+...
              (1-costheta)/(tp2)*Omegahatp2+...
              ((theta-sintheta)/(tp3))*Omegahatp3;
        T  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+f(1)*adjOmegap(1:6,:)+...
             f(2)*adjOmegap(7:12,:)+...
             f(3)*adjOmegap(13:18,:)+...
             f(4)*adjOmegap(19:24,:);
        Td = fd(1)*thetad*adjOmegap(1:6,:)+f(1)*adjOmegapd(1:6,:)+...
             fd(2)*thetad*adjOmegap(7:12,:)+f(2)*adjOmegapd(7:12,:)+...
             fd(3)*thetad*adjOmegap(13:18,:)+f(3)*adjOmegapd(13:18,:)+...
             fd(4)*thetad*adjOmegap(19:24,:)+f(4)*adjOmegapd(19:24,:);
    end

%% computation of dSdq_qd, dSdq_qdd, dSddq_qd, dSddqd_qd

    S  = T*Z;
    TZd = T*Zd;
    Sd = Td*Z+TZd;

    dSdq_qd  = TZd;
    dSdq_qdd = T*Zdd;
    dSddq_qd = Td*Zd;
    
    Zqdd = Z*qdd;
    Zdqd = Zd*qd;
    if (theta<=1e-2)
        for r=1:4 %1st for loop
            %nothing here as fd and fdd are zero
            for u=1:r %2nd for loop
                
                if u==1
                    adjOmegap1 = eye(6);
                else
                    adjOmegap1 = adjOmegap((u-2)*6+1:6*(u-1),:); %adjOmega^(u-1)
                end
                
                if u==r
                    adjOmegap2 = eye(6);
                else
                    adjOmegap2 = adjOmegap((r-u-1)*6+1:6*(r-u),:); %adjOmega^(r-u)
                end
                
                dSdq_qd  = dSdq_qd-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Omegad)*Z;
                dSdq_qdd = dSdq_qdd-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Zqdd)*Z;
                dSddq_qd = dSddq_qd-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Zdqd)*Z-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Omegad)*Zd;
    
                for p=1:u-1 %3rd for loop
    
                    if p==1
                        adjOmegap3 = eye(6);
                    else
                        adjOmegap3 = adjOmegap((p-2)*6+1:6*(p-1),:); %adjOmega^(p-1)
                    end
                    if p==u-1
                        adjOmegap4 = eye(6);
                    else
                        adjOmegap4 = adjOmegap((u-p-2)*6+1:6*(u-p-1),:); %adjOmega^(u-p-1)
                    end
                    dSddq_qd = dSddq_qd-f(r)*adjOmegap3*dinamico_adj(adjOmegap4*adjOmegapd(1:6,:)*adjOmegap2*Omegad)*Z;
    
                end
    
                for p=1:r-u %3rd for loop
    
                    if p==1
                        adjOmegap3 = eye(6);
                    else
                        adjOmegap3 = adjOmegap((p-2)*6+1:6*(p-1),:); %adjOmega^(p-1)
                    end
                    if p==r-u
                        adjOmegap4 = eye(6);
                    else
                        adjOmegap4 = adjOmegap((r-u-p-1)*6+1:6*(r-u-p),:); %adjOmega^(r-u-p)
                    end
                    dSddq_qd = dSddq_qd-f(r)*adjOmegap1*adjOmegapd(1:6,:)*adjOmegap3*dinamico_adj(adjOmegap4*Omegad)*Z;
    
                end
    
            end
        end
    else
        for r=1:4 %1st for loop
    
            dSdq_qd  = dSdq_qd+(1/theta)*fd(r)*adjOmegap((r-1)*6+1:6*r,:)*Omegad*Omega'*I_theta*Z;
            dSdq_qdd = dSdq_qdd+(1/theta)*fd(r)*adjOmegap((r-1)*6+1:6*r,:)*Zqdd*Omega'*I_theta*Z;
            dSddq_qd = dSddq_qd+(1/theta)*fd(r)*adjOmegap((r-1)*6+1:6*r,:)*Zdqd*Omega'*I_theta*Z...
                               +(1/theta)*(fdd(r)*thetad*adjOmegap((r-1)*6+1:6*r,:)+fd(r)*adjOmegapd((r-1)*6+1:6*r,:))*Omegad*Omega'*I_theta*Z...
                               +(1/theta)*fd(r)*adjOmegap((r-1)*6+1:6*r,:)*Omegad*((Omegad'-thetad/theta*Omega')*I_theta*Z+Omega'*I_theta*Zd);
            for u=1:r %2nd for loop
                
                if u==1
                    adjOmegap1 = eye(6);
                else
                    adjOmegap1 = adjOmegap((u-2)*6+1:6*(u-1),:); %adjOmega^(u-1)
                end
                
                if u==r
                    adjOmegap2 = eye(6);
                else
                    adjOmegap2 = adjOmegap((r-u-1)*6+1:6*(r-u),:); %adjOmega^(r-u)
                end
                
                dSdq_qd  = dSdq_qd-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Omegad)*Z;
                dSdq_qdd = dSdq_qdd-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Zqdd)*Z;
                dSddq_qd = dSddq_qd-f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Zdqd)*Z...
                                   -fd(r)*thetad*adjOmegap1*dinamico_adj(adjOmegap2*Omegad)*Z...
                                   -f(r)*adjOmegap1*dinamico_adj(adjOmegap2*Omegad)*Zd;
    
                for p=1:u-1 %3rd for loop
    
                    if p==1
                        adjOmegap3 = eye(6);
                    else
                        adjOmegap3 = adjOmegap((p-2)*6+1:6*(p-1),:); %adjOmega^(p-1)
                    end
                    if p==u-1
                        adjOmegap4 = eye(6);
                    else
                        adjOmegap4 = adjOmegap((u-p-2)*6+1:6*(u-p-1),:); %adjOmega^(u-p-1)
                    end
                    dSddq_qd = dSddq_qd-f(r)*adjOmegap3*dinamico_adj(adjOmegap4*adjOmegapd(1:6,:)*adjOmegap2*Omegad)*Z;
    
                end
    
                for p=1:r-u %3rd for loop
    
                    if p==1
                        adjOmegap3 = eye(6);
                    else
                        adjOmegap3 = adjOmegap((p-2)*6+1:6*(p-1),:); %adjOmega^(p-1)
                    end
                    if p==r-u
                        adjOmegap4 = eye(6);
                    else
                        adjOmegap4 = adjOmegap((r-u-p-1)*6+1:6*(r-u-p),:); %adjOmega^(r-u-p)
                    end
                    dSddq_qd = dSddq_qd-f(r)*adjOmegap1*adjOmegapd(1:6,:)*adjOmegap3*dinamico_adj(adjOmegap4*Omegad)*Z;
    
                end
    
            end
        end
    end

end