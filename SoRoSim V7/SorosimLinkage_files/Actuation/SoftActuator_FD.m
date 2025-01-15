function Phi_au = SoftActuator_FD(u,dc,dcp,xihat_123)

dtilde = dinamico_tilde(dc);
tang   = xihat_123*[dc;1]+dcp;
ntang  = norm(tang);
tang   = tang/ntang;
Phi_au = u*[dtilde*tang;tang];

end