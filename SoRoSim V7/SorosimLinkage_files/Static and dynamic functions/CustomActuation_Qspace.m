%Function to convert user defined actuation force Fact and its Jacobian wrt q on soft links to actuation force in Q space
%Last modified by Anup Teejo Mathew - 05/02/2025
function [tauC,dtauC_dq] = CustomActuation_Qspace(Linkage,Fact,dFact_dq)

ndof = Linkage.ndof;
N    = Linkage.N;

tauC     = zeros(ndof,1); %change here
dtauC_dq = zeros(ndof,ndof); %change here
i_sig_s = 1;

dof_start=1;
for i=1:N
    
    VRods   = Linkage.CVRods{i};
    dof_start = dof_start+VRods(1).dof;
    dofs_here = dof_start:dof_start+dof_here-1;
    
    for j=1:Linkage.VLinks(i).npie-1
        dof_here     = VRods(j+1).dof;
        Ws  = Linkage.CVRods{i}(j+1).Ws;
        nip = Linkage.CVRods{i}(j+1).nip;
        for ii=1:nip
            if Ws(ii)>0
                Fact_here = Fact((i_sig_s-1)*6+1:i_sig_s*6);
                Phi_here    = VRods(j+1).Phi((ii-1)*6+1:ii*6,:);

                tauC(dofs_here)       = TauC(dofs_here)+Ws(ii)*Phi_here'*Fact_here;
                dtauC_dq(dofs_here,:) = dtauC_dq(dofs_here,:)+Ws(ii)*Phi_here'*dFact_dq;
            end
            i_sig_s = i_sig_s+1;
        end
        dof_start = dof_start+dof_here;
    end
end

end

