%Computes the generalized damping matrix for the linkage
%Last modified by Anup Teejo Mathew 05.12.2024
function D = findD(S)

dof_start = 1;

D = zeros(S.ndof,S.ndof); %ndofxndof matrix 

for i=1:S.N

    VRods = S.CVRods{i};
    dof_here = VRods(1).dof;

    %for joint
    dofs_here = dof_start:dof_start+dof_here-1;
    Phi  = VRods(1).Phi;
    D(dofs_here,dofs_here) = D(dofs_here,dofs_here)+Phi'*VRods(1).Gs*Phi;
        
    
    if ~S.OneBasis
        dof_start = dof_start+dof_here;
    end

    for j=1:(S.VLinks(S.LinkIndex(i)).npie)-1 %for the rest of soft pieces

        Gs       = VRods(j+1).Gs;
        dof_here = VRods(j+1).dof;
        Ws       = VRods(j+1).Ws;
        nip      = VRods(j+1).nip;

        Dtemp  = zeros(dof_here,dof_here);
        dofs_here = dof_start:dof_start+dof_here-1;
        Phi  = VRods(j+1).Phi;

        for ii=1:nip
            if Ws(ii)>0
                Gs_here = Gs((ii-1)*6+1:ii*6,:);
                Dtemp = Dtemp+Ws(ii)*Phi((ii-1)*6+1:ii*6,:)'*Gs_here*Phi((ii-1)*6+1:ii*6,:);
            end
        end

        D(dofs_here,dofs_here) =D(dofs_here,dofs_here)+Dtemp;

        if ~S.OneBasis
            dof_start = dof_start+dof_here;
        end

    end
end

end
