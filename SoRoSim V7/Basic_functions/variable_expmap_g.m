function g=variable_expmap_g(Omega)

k     = Omega(1:3);
theta = norm(k);

Omegahat   = dinamico_hat(Omega);
Omegahatp2 = Omegahat*Omegahat;
Omegahatp3 = Omegahatp2*Omegahat;

if (theta<=1e-2)
    g  = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+Omegahatp2/2+Omegahatp3/6;
else

    tp2 = theta*theta;
    tp3 = tp2*theta;
    
    sintheta = sin(theta);
    costheta = cos(theta);
    
    g   = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]+Omegahat+...
          (1-costheta)/(tp2)*Omegahatp2+...
          ((theta-sintheta)/(tp3))*Omegahatp3;
end
