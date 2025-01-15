%Computes the generalized stiffness matrix for the linkage
%Last modified by Anup Teejo Mathew 14.04.2023
function K = findK(S) %NxN matrix for independent basis, NX1 for dependent basis if any

dof_start = 1;

K = zeros(S.ndof,S.ndof); %ndofxndof matrix 
for i=1:S.N

    VRods = S.CVRods{i};
    dof_here = VRods(1).dof;
    
    %for joint
    dofs_here = dof_start:dof_start+dof_here-1;
    Phi  = VRods(1).Phi;
    K(dofs_here,dofs_here) = K(dofs_here,dofs_here)+Phi'*VRods(1).Es*Phi;

    if ~S.OneBasis
        dof_start = dof_start+dof_here;
    end
 
    for j=1:(S.VLinks(S.LinkIndex(i)).npie)-1 %for the rest of soft link divisions

        Es       = VRods(j+1).Es;
        dof_here = VRods(j+1).dof;
        Ws       = VRods(j+1).Ws;
        nip      = VRods(j+1).nip;

        Ktemp  = zeros(dof_here,dof_here);
        dofs_here = dof_start:dof_start+dof_here-1;
        Phi = VRods(j+1).Phi;

        for ii=1:nip
            if Ws(ii)>0
                Es_here = Es((ii-1)*6+1:ii*6,:);
                Ktemp = Ktemp+Ws(ii)*Phi((ii-1)*6+1:ii*6,:)'*Es_here*Phi((ii-1)*6+1:ii*6,:);
            end
        end
        K(dofs_here,dofs_here) =K(dofs_here,dofs_here)+Ktemp;

        if ~S.OneBasis
            dof_start = dof_start+dof_here;
        end
    end
end

end