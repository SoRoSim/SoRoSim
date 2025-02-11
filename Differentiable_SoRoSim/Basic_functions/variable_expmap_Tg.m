function Tg = variable_expmap_Tg(Omega)

theta    = norm(Omega(4:6));
adjOmega = dinamico_adj(Omega);

adjOmegap2 = adjOmega*adjOmega;
adjOmegap3 = adjOmegap2*adjOmega;
adjOmegap4 = adjOmegap3*adjOmega;

if (theta<=1e-2)
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
    
    Tg  = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1]+((4-4*costheta-t1)/(2*tp2))*adjOmega+...
          ((4*theta-5*sintheta+t2)/(2*tp3))*adjOmegap2+...
          ((2-2*costheta-t1)/(2*tp4))*adjOmegap3+...
          ((2*theta-3*sintheta+t2)/(2*tp5))*adjOmegap4;
end

% eof