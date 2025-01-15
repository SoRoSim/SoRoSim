function Linkage = ClosedLoopRedundancyPrecompute(Linkage)
    
    J = Linkage.Jacobian(zeros(Linkage.ndof,1)); %Jacobian
    g = Linkage.FwdKinematics(zeros(Linkage.ndof,1));
    ndof = Linkage.ndof;
    nCLp = 0;

    for iCLj=1:Linkage.nCLj

        i_sigA = Linkage.CLprecompute.i_sigA(iCLj);
        i_sigB = Linkage.CLprecompute.i_sigB(iCLj);
        Phi_p = Linkage.CLprecompute.Phi_p{iCLj};
        % nl = size(Phi_p,2);
        
        if i_sigA>0
            if Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).linktype=='s'
                g_sigA2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).gf{end}*Linkage.gACLj{iCLj};
            else
                g_sigA2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).gf*Linkage.gACLj{iCLj};
            end
            gA = g((i_sigA-1)*4+1:i_sigA*4,:)*g_sigA2CLj;
            JA = dinamico_Adjoint(ginv(g_sigA2CLj))*J((i_sigA-1)*6+1:i_sigA*6,:);
        else %if ground
            gA = Linkage.gACLj{iCLj};
            JA = zeros(6,ndof);
        end
    
        if i_sigB>0
            if Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).linktype=='s'
                g_sigB2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).gf{end}*Linkage.gBCLj{iCLj};
            else
                g_sigB2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).gf*Linkage.gBCLj{iCLj};
            end
            gB = g((i_sigB-1)*4+1:i_sigB*4,:)*g_sigB2CLj;
            JB = dinamico_Adjoint(ginv(g_sigB2CLj))*J((i_sigB-1)*6+1:i_sigB*6,:);
        else %if ground
            gB = Linkage.gBCLj{iCLj};
            JB = zeros(6,ndof);
        end
    
        gBA = ginv(gB)*gA; %typically Identity
        OmegaBA = piecewise_logmap(gBA); %twist vector from B to A (typically zero)
        Adj_gBA = dinamico_Adjoint(gBA);
        diffJAB = Adj_gBA*JA-JB; %in B frame
        [~,T_OmegaBA] = variable_expmap_gTg_mex(OmegaBA); %Tangent operator of the twist vector
    
        de_dq_CLj   = Phi_p'*(T_OmegaBA\diffJAB); %Jacobian of e
        
        if Linkage.Actuated
            [~,iRows] = LIRows(de_dq_CLj(Linkage.ActuationPrecompute.index_q_u)); %Linearly independent rows
        else
            [~,iRows] = LIRows(de_dq_CLj); %Linearly independent rows
        end
        
        Phi_p = Phi_p(:,iRows);
        Linkage.CLprecompute.Phi_p{iCLj} = Phi_p;
        nCLp = nCLp+size(Phi_p,2);
    end

    Linkage.CLprecompute.nCLp = nCLp;

end