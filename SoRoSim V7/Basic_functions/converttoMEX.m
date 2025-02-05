%Either create variable size or redo for all problem, different dof require
%different varialble size. Variable size takes more time

h          = coder.typeof(0); % h is a scalar double
xi_star    = coder.typeof(zeros(6,1)); 
xi_star_Z1 = coder.typeof(zeros(6,1)); 
xi_star_Z2 = coder.typeof(zeros(6,1));
Omega      = coder.typeof(zeros(6,1));
Omegad     = coder.typeof(zeros(6,1));
T          = coder.typeof(zeros(6,6));
f          = coder.typeof(zeros(4,1));
fd         = coder.typeof(zeros(4,1));
adjOmegap  = coder.typeof(zeros(24,6));
F_C        = coder.typeof(zeros(6,1));
ndof       = coder.typeof(0); % ndof is a scalar double

max_ndof = 100; % Maximum DoF of a Cosserat Rod

Phi    = coder.typeof(zeros(6, max_ndof), [6, max_ndof], [0, 1]); % 6 x ndof with variable ndof
Phi_Z  = coder.typeof(zeros(6, max_ndof), [6, max_ndof], [0, 1]); % 6 x ndof with variable ndof
Phi_Z1 = coder.typeof(zeros(6, max_ndof), [6, max_ndof], [0, 1]); % 6 x ndof with variable ndof
Phi_Z2 = coder.typeof(zeros(6, max_ndof), [6, max_ndof], [0, 1]); % 6 x ndof with variable ndof
Z      = coder.typeof(zeros(6, max_ndof), [6, max_ndof], [0, 1]); % 6 x ndof with variable ndof
q      = coder.typeof(zeros(max_ndof, 1), [max_ndof, 1], [1, 0]); % ndof x 1 with variable ndof
qd     = coder.typeof(zeros(max_ndof, 1), [max_ndof, 1], [1, 0]); % ndof x 1 with variable ndof
qdd    = coder.typeof(zeros(max_ndof, 1), [max_ndof, 1], [1, 0]); % ndof x 1 with variable ndof

% Generate MEX file
codegen variable_expmap_gTgTgd -args {Omega, Omegad}
codegen variable_expmap_gTg -args {Omega}
codegen variable_expmap_g -args {Omega}

codegen RigidJointKinematics -args {Phi,xi_star,q} %very close to SoftJointKinematics_Z2
codegen RigidJointDifferentialKinematics -args {Phi,xi_star,q,qd,qdd}

codegen SoftJointKinematics_Z4 -args {h,Phi_Z1,Phi_Z2,xi_star_Z1,xi_star_Z2,q}
codegen SoftJointDifferentialKinematics_Z4 -args {h,Phi_Z1,Phi_Z2,xi_star_Z1,xi_star_Z2,q,qd,qdd}

codegen SoftJointKinematics_Z2 -args {h,Phi_Z,xi_star,q}
codegen SoftJointDifferentialKinematics_Z2 -args {h,Phi_Z,xi_star,q,qd,qdd}

codegen compute_dSTdqFC_Z4 -args {h, Omega, Phi_Z1, Phi_Z2 ,Z, T, f, fd, adjOmegap, F_C}
codegen compute_dSTdqFC_Z2R -args {ndof, Omega, Z, f, fd, adjOmegap, F_C}



