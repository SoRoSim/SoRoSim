%Either create variable size or redo for all problem, different dof require
%different varialble size. Variable size takes more time

Omega = coder.typeof(zeros(6,1));
Omegad = coder.typeof(zeros(6,1));

% Generate MEX file
codegen variable_expmap_gTgTgd -args {Omega, Omegad}
codegen variable_expmap_gTg -args {Omega}
codegen variable_expmap_g -args {Omega}




