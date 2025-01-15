%Function to convert user defined actuation force (Fact) on soft links to actuation force in Q space
%Last modified by Anup Teejo Mathew - 23/06/2021
function [tauC,dtauC_dq] = CustomActuation_Qspace(Tr,Fact,dFact_dq)

ndof = Tr.ndof;
N    = Tr.N;

tauC     = zeros(ndof,1); %change here
dtauC_dq = zeros(ndof,ndof); %change here
i_sig_s = 1;

dof_start=1;
for i=1:N
    
    VRods   = Tr.CVTwists{i};
    dof_start = dof_start+VRods(1).dof;
    
    for j=1:Tr.VLinks(i).npie-1
        dof_here     = VRods(j+1).dof;
        Ws  = Tr.CVTwists{i}(j+1).Ws;
        nip = Tr.CVTwists{i}(j+1).nip;
        for ii=1:nip
            if Ws(ii)>0
                Fact_here = Fact((i_sig_s-1)*6+1:i_sig_s*6);
                B_here    = VRods(j+1).B((ii-1)*6+1:ii*6,:);

                tauC(dof_start:dof_start+dof_here-1) = TauC(dof_start:dof_start+dof_here-1)+Ws(ii)*B_here'*Fact_here;
                dtauC_dq(dof_start:dof_start+dof_here-1,:) = TauC(dof_start:dof_start+dof_here-1)+Ws(ii)*B_here'*dFact_dq;
            end
            i_sig_s = i_sig_s+1;
        end
        dof_start = dof_start+dof_here;
    end
end

end

