function Phi_a = SoftActuator_FD(dc,dcp,xihat_123)

dtilde = dinamico_tilde(dc);
tang   = xihat_123*[dc;1]+dcp;
ntang  = norm(tang);
tang   = tang/ntang;
Phi_a = [dtilde*tang;tang];

end