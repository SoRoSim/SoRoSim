function [Omega,Z,g,T,S,f,fd,adjOmegap] = SoftJointKinematics_Z4(h,Phi_Z1,Phi_Z2,xi_star_Z1,xi_star_Z2,q)
%Phi is Phi_scale*Phi_Leg. New Scaleing method!
    
    
    Zc      = sqrt(3)/12;
    hp2     = h*h;
    Zchp2   = Zc*hp2;
    
    if isempty(q)
        xi_Z1 = xi_star_Z1;
        xi_Z2 = xi_star_Z2;
    else
        xi_Z1 = Phi_Z1*q+xi_star_Z1;
        xi_Z2 = Phi_Z2*q+xi_star_Z2;
    end

    Omega   = h/2*(xi_Z1+xi_Z2)+Zchp2*dinamico_adj(xi_Z1)*xi_Z2;
    Z       = h/2*(Phi_Z1+Phi_Z2)+Zchp2*(dinamico_adj(xi_Z1)*Phi_Z2-dinamico_adj(xi_Z2)*Phi_Z1);

    %% defining T, Td, f, fd, fdd, adjOmegap (powers of adjOmega)

    f   = zeros(4,1);
    fd  = zeros(4,1);

    adjOmegap  = zeros(24,6); %all powers of adjGamma || INCLUDE POWER OF 0 to avoid an if

    k      = Omega(1:3);
    theta  = norm(k);
    
    Omegahat  = dinamico_hat(Omega);
    Omegahatp2 = Omegahat*Omegahat;
    Omegahatp3 = Omegahatp2*Omegahat;

    adjOmegap(1:6,:)  = dinamico_adj(Omega);
    adjOmegap(7:12,:)  = adjOmegap(1:6,:)*adjOmegap(1:6,:);
    adjOmegap(13:18,:) = adjOmegap(7:12,:)*adjOmegap(1:6,:);
    adjOmegap(19:24,:) = adjOmegap(13:18,:)*adjOmegap(1:6,:);

    if (theta<=1e-2)
        g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+Omegahatp2/2+Omegahatp3/6;

        f(1) = 1/2;
        f(2) = 1/6;
        f(3) = 1/24;
        f(4) = 1/120;

        T  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+f(1)*adjOmegap(1:6,:)+f(2)*adjOmegap(7:12,:)+f(3)*adjOmegap(13:18,:)+f(4)*adjOmegap(19:24,:);
    else
 
        tp2        = theta*theta;
        tp3        = tp2*theta;
        tp4        = tp3*theta;
        tp5        = tp4*theta;
        tp6        = tp5*theta;
        
        sintheta   = sin(theta);
        costheta   = cos(theta);
        
        t1 = theta*sintheta;
        t2 = theta*costheta;
        t3 = -8+(8-tp2)*costheta+5*t1;
        t4 = -8*theta+(15-tp2)*sintheta-7*t2;
    
        f(1) = (4-4*costheta-t1)/(2*tp2);
        f(2) = (4*theta-5*sintheta+t2)/(2*tp3);
        f(3) = (2-2*costheta-t1)/(2*tp4);
        f(4) = (2*theta-3*sintheta+t2)/(2*tp5);
    
        fd(1) = (t3)/(2*tp3);
        fd(2) = (t4)/(2*tp4);
        fd(3) = (t3)/(2*tp5);
        fd(4) = (t4)/(2*tp6);
    
        
        g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+...
              (1-costheta)/(tp2)*Omegahatp2+...
              ((theta-sintheta)/(tp3))*Omegahatp3;
        T  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+f(1)*adjOmegap(1:6,:)+...
             f(2)*adjOmegap(7:12,:)+...
             f(3)*adjOmegap(13:18,:)+...
             f(4)*adjOmegap(19:24,:);
    end

%% computation of S

    S  = T*Z;

end