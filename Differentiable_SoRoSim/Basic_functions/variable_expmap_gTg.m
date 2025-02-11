function [g,Tg]=variable_expmap_gTg(Omega)

k      = Omega(1:3);
theta  = norm(k);

Omegahat  = dinamico_hat(Omega);
adjOmega  = dinamico_adj(Omega);

Omegahatp2 = Omegahat*Omegahat;
Omegahatp3 = Omegahatp2*Omegahat;

adjOmegap2 = adjOmega*adjOmega;
adjOmegap3 = adjOmegap2*adjOmega;
adjOmegap4 = adjOmegap3*adjOmega;

if (theta<=1e-2)
    g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+Omegahatp2/2+Omegahatp3/6;
    
    f1 = 1/2;
    f2 = 1/6;
    f3 = 1/24;
    f4 = 1/120;
    
    Tg  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+f1*adjOmega+f2*adjOmegap2+f3*adjOmegap3+f4*adjOmegap4;
else
    
    tp2 = theta*theta;
    tp3 = tp2*theta;
    tp4 = tp3*theta;
    tp5 = tp4*theta;
    
    sintheta = sin(theta);
    costheta = cos(theta);
    
    t1 = theta*sintheta;
    t2 = theta*costheta;
    
    g   = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+...
          (1-costheta)/(tp2)*Omegahatp2+...
          ((theta-sintheta)/(tp3))*Omegahatp3;
    Tg  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+((4-4*costheta-t1)/(2*tp2))*adjOmega+...
          ((4*theta-5*sintheta+t2)/(2*tp3))*adjOmegap2+...
          ((2-2*costheta-t1)/(2*tp4))*adjOmegap3+...
          ((2*theta-3*sintheta+t2)/(2*tp5))*adjOmegap4;
end
