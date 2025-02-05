function [g,Tg,Tgd]=variable_expmap_gTgTgd(Omega,Omegad)

k      = Omega(1:3);
theta  = norm(k);

Omegahat  = dinamico_hat(Omega);
adjOmega  = dinamico_adj(Omega);
adjOmegad = dinamico_adj(Omegad);

Omegahatp2 = Omegahat*Omegahat;
Omegahatp3 = Omegahatp2*Omegahat;

adjOmegap2 = adjOmega*adjOmega;
adjOmegap3 = adjOmegap2*adjOmega;
adjOmegap4 = adjOmegap3*adjOmega;

adjOmegad2 = adjOmegad*adjOmega+adjOmega*adjOmegad;
adjOmegad3 = adjOmegad2*adjOmega+adjOmegap2*adjOmegad;
adjOmegad4 = adjOmegad3*adjOmega+adjOmegap3*adjOmegad;


if (theta<=1e-2)
    g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+Omegahatp2/2+Omegahatp3/6;
    
    f1 = 1/2;
    f2 = 1/6;
    f3 = 1/24;
    f4 = 1/120;
    
    Tg  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+f1*adjOmega+f2*adjOmegap2+f3*adjOmegap3+f4*adjOmegap4;
    Tgd = f1*adjOmegad+f2*adjOmegad2+f3*adjOmegad3+f4*adjOmegad4;
else

    kd     = Omegad(1:3);
    thetad = (kd'*k)/theta;

    tp2 = theta*theta;
    tp3 = tp2*theta;
    tp4 = tp3*theta;
    tp5 = tp4*theta;
    tp6 = tp5*theta;
    
    sintheta = sin(theta);
    costheta = cos(theta);
    
    t1 = theta*sintheta;
    t2 = theta*costheta;
    t3 = -8+(8-tp2)*costheta+5*t1;
    t4 = -8*theta+(15-tp2)*sintheta-7*t2;
    t5 = (4-4*costheta-t1)/(2*tp2);
    t6 = (4*theta-5*sintheta+t2)/(2*tp3);
    t7 = (2-2*costheta-t1)/(2*tp4);
    t8 = (2*theta-3*sintheta+t2)/(2*tp5);
    
    g   = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+...
          (1-costheta)/(tp2)*Omegahatp2+...
          ((theta-sintheta)/(tp3))*Omegahatp3;
    Tg  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+t5*adjOmega+...
          t6*adjOmegap2+...
          t7*adjOmegap3+...
          t8*adjOmegap4;
    Tgd = (thetad*(t3)/(2*tp3))*adjOmega+t5*adjOmegad+...
          (thetad*(t4)/(2*tp4))*adjOmegap2+t6*adjOmegad2+...
          (thetad*(t3)/(2*tp5))*adjOmegap3+t7*adjOmegad3+...
          (thetad*(t4)/(2*tp6))*adjOmegap4+t8*adjOmegad4;
end
