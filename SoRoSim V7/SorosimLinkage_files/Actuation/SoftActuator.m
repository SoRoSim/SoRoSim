function [Phi_a,PHI_AU] = SoftActuator(u,dc,dcp,xihat_123)

dtilde = dinamico_tilde(dc);
tang   = xihat_123*[dc;1]+dcp;
ntang  = norm(tang);
tang   = tang/ntang;
tangtilde = dinamico_tilde(tang);
tangtildep2 = tangtilde*tangtilde;
tangtildep2dtilde = tangtildep2*dtilde;
Phi_a = [dtilde*tang;tang];
PHI_AU = u/ntang*[dtilde*tangtildep2dtilde tangtildep2dtilde';tangtildep2dtilde -tangtildep2];

end