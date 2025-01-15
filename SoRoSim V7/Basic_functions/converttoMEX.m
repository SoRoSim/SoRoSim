%Either create variable size or redo for all problem, different dof require
%different varialble size. Variable size takes more time

Gamma = coder.typeof(zeros(6,1));
Gammad = coder.typeof(zeros(6,1));

% Generate MEX file
codegen variable_expmap_gTgTgd -args {Gamma, Gammad}
codegen variable_expmap_gTg -args {Gamma}
codegen variable_expmap_g -args {Gamma}




