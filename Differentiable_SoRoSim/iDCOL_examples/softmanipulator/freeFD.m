function qdd = freeFD(Linkage, q, qd, u)

%single pass FD algorithm. replace with ABA in future

ndof = Linkage.ndof;
N    = Linkage.N;
nsig = Linkage.nsig;

M = zeros(ndof,ndof);
F = zeros(ndof,1);

g   = zeros(4*nsig,4);
J   = zeros(6*nsig,ndof);
Jd  = zeros(6*nsig,ndof);

dof_start = 1;
i_sig     = 1;

g_ini    = Linkage.g_ini;
g_tip    = repmat(eye(4),N,1);
J_tip    = repmat(zeros(6,ndof),N,1);
Jd_tip   = repmat(zeros(6,ndof),N,1);
eta_tip  = zeros(N*6,1);
psi_tip  = zeros(N*6,1);
iLpre    = Linkage.iLpre;

for i=1:N

    G = Linkage.G;
    if ~Linkage.Gravity
        G = G*0;
    elseif Linkage.UnderWater
        G = G*(1-Linkage.Rho_water/Linkage.VLinks(Linkage.LinkIndex(i)).Rho);
    end

    if iLpre(i)>0
        g_here       = g_tip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        Ad_g_ini_inv = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)));
        J_here       = Ad_g_ini_inv*J_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        Jd_here      = Ad_g_ini_inv*Jd_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        eta_here     = Ad_g_ini_inv*eta_tip((iLpre(i)-1)*6+1:iLpre(i)*6);
        psi_here     = Ad_g_ini_inv*psi_tip((iLpre(i)-1)*6+1:iLpre(i)*6);
    else
        g_here   = g_ini((i-1)*4+1:i*4,:);
        J_here   = zeros(6,ndof);
        Jd_here  = zeros(6,ndof);
        eta_here = zeros(6,1);
        psi_here = zeros(6,1);
    end

    g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
    J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
    Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
    i_sig                        = i_sig+1;

    dof_here  = Linkage.CVRods{i}(1).dof;
    dofs_here = dof_start:dof_start+dof_here-1;
    q_here    = q(dofs_here);
    qd_here   = qd(dofs_here);
    Phi_here  = Linkage.CVRods{i}(1).Phi;
    xi_star   = Linkage.CVRods{i}(1).xi_star;

    S_here  = zeros(6,ndof);
    Sd_here = zeros(6,ndof);

    if dof_here==0
        gstep = eye(4);
    else
        xi  = Phi_here*q_here+xi_star;
        xid = Phi_here*qd_here;

        [gstep,Tg,Tgd] = variable_expmap_gTgTgd_mex(xi,xid);

        S_here(:,dofs_here)  = Tg*Phi_here;
        Sd_here(:,dofs_here) = Tgd*Phi_here;
    end

    g_here         = g_here*gstep;
    Ad_g_joint_inv = dinamico_Adjoint(ginv(gstep));
    J_here         = Ad_g_joint_inv*(J_here+S_here);
    Jd_here        = Ad_g_joint_inv*(Jd_here+Sd_here+dinamico_adj(eta_here)*S_here);
    psi_here       = Ad_g_joint_inv*(psi_here+(Sd_here(:,dofs_here)+dinamico_adj(eta_here)*S_here(:,dofs_here))*qd_here);
    eta_here       = Ad_g_joint_inv*(eta_here+S_here(:,dofs_here)*qd_here);

    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'

        gi        = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
        g_here    = g_here*gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        J_here    = Ad_gi_inv*J_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        psi_here  = Ad_gi_inv*psi_here;

        g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
        J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
        Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;

        M_here = Linkage.VLinks(Linkage.LinkIndex(i)).M;
        F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;

        if Linkage.UnderWater
            M_here  = M_here+Linkage.M_added((i_sig-1)*6+1:i_sig*6,:);
            DL_here = Linkage.DL((i_sig-1)*6+1:i_sig*6,:);
            v_here  = norm(eta_here(4:6));
            F = F-J_here'*DL_here*v_here*eta_here;
        end

        i_sig = i_sig+1;

        Qtemp = J_here'*M_here;
        M = M+Qtemp*J_here;
        F = F-Qtemp*psi_here-J_here'*dinamico_coadj(eta_here)*M_here*eta_here;

        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        psi_here  = Ad_gf_inv*psi_here;

    end

    if ~Linkage.OneBasis
        dof_start = dof_start+dof_here;
    end

    for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1

        dof_here  = Linkage.CVRods{i}(j+1).dof;
        dofs_here = dof_start:dof_start+dof_here-1;
        q_here    = q(dofs_here);
        qd_here   = qd(dofs_here);

        gi      = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        ld      = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        xi_star = Linkage.CVRods{i}(j+1).xi_star;
        Ms      = Linkage.CVRods{i}(j+1).Ms;
        Xs      = Linkage.CVRods{i}(j+1).Xs;
        Ws      = Linkage.CVRods{i}(j+1).Ws;
        nip     = Linkage.CVRods{i}(j+1).nip;

        g_here    = g_here*gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        J_here    = Ad_gi_inv*J_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        psi_here  = Ad_gi_inv*psi_here;

        g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
        J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
        Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;

        ii = 1;
        if Ws(ii)>0
            M_here = Ws(ii)*Ms(6*(ii-1)+1:6*ii,:);
            F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;

            if Linkage.UnderWater
                M_here  = M_here+Ws(ii)*Linkage.M_added((i_sig-1)*6+1:i_sig*6,:);
                DL_here = Ws(ii)*Linkage.DL((i_sig-1)*6+1:i_sig*6,:);
                v_here  = norm(eta_here(4:6));
                F = F-J_here'*DL_here*v_here*eta_here;
            end

            Qtemp = J_here'*M_here;
            M = M+Qtemp*J_here;
            F = F-Qtemp*psi_here-J_here'*dinamico_coadj(eta_here)*M_here*eta_here;
        end
        i_sig = i_sig+1;

        for ii=2:nip

            H = (Xs(ii)-Xs(ii-1))*ld;

            if Linkage.Z_order==4

                fZ4 = ((sqrt(3)*H^2)/12);

                xi_Z1here  = xi_star(6*(ii-2)+1:6*(ii-1),2);
                xi_Z2here  = xi_star(6*(ii-2)+1:6*(ii-1),3);
                Phi_Z1here = Linkage.CVRods{i}(j+1).Phi_Z1(6*(ii-2)+1:6*(ii-1),:);
                Phi_Z2here = Linkage.CVRods{i}(j+1).Phi_Z2(6*(ii-2)+1:6*(ii-1),:);

                if dof_here>0
                    xi_Z1here    = Phi_Z1here*q_here+xi_Z1here;
                    xi_Z2here    = Phi_Z2here*q_here+xi_Z2here;
                    xid_Z1here   = Phi_Z1here*qd_here;
                    ad_xi_Z1here = dinamico_adj(xi_Z1here);
                    Z_here       = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                                   fZ4*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);
                    Zd_here      = 2*fZ4*dinamico_adj(xid_Z1here)*Phi_Z2here;
                    Omegad_here  = Z_here*qd_here;
                else
                    ad_xi_Z1here = dinamico_adj(xi_Z1here);
                    Z_here       = [];
                    Zd_here      = [];
                    Omegad_here  = zeros(6,1);
                end

                Omega_here = (H/2)*(xi_Z1here+xi_Z2here)+...
                             fZ4*ad_xi_Z1here*xi_Z2here;

            else

                xi_Zhere  = xi_star(6*(ii-2)+1:6*(ii-1),4);
                Phi_Zhere = Linkage.CVRods{i}(j+1).B_Z(6*(ii-2)+1:6*(ii-1),:);

                if dof_here>0
                    xi_Zhere    = Phi_Zhere*q_here+xi_Zhere;
                    Z_here      = H*Phi_Zhere;
                    Omegad_here = Z_here*qd_here;
                else
                    Z_here      = H*Phi_Zhere;
                    Omegad_here = zeros(6,1);
                end
                Omega_here = H*xi_Zhere;

            end

            [gstep,Tg,Tgd] = variable_expmap_gTgTgd_mex(Omega_here,Omegad_here);

            S_here  = zeros(6,ndof);
            Sd_here = zeros(6,ndof);

            if dof_here>0
                S_here(:,dofs_here)  = Tg*Z_here;
                Sd_here(:,dofs_here) = Tgd*Z_here;
                if Linkage.Z_order==4
                    Sd_here(:,dofs_here) = Sd_here(:,dofs_here)+Tg*Zd_here;
                end
            end

            g_here    = g_here*gstep;
            Ad_gh_inv = dinamico_Adjoint(ginv(gstep));
            J_here    = Ad_gh_inv*(J_here+S_here);
            Jd_here   = Ad_gh_inv*(Jd_here+Sd_here+dinamico_adj(eta_here)*S_here);
            psi_here  = Ad_gh_inv*(psi_here+(Sd_here(:,dofs_here)+dinamico_adj(eta_here)*S_here(:,dofs_here))*qd_here);
            eta_here  = Ad_gh_inv*(eta_here+S_here(:,dofs_here)*qd_here);

            g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
            J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
            Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;

            if Ws(ii)>0
                M_here = Ws(ii)*Ms(6*(ii-1)+1:6*ii,:);
                F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;

                if Linkage.UnderWater
                    M_here  = M_here+Ws(ii)*Linkage.M_added((i_sig-1)*6+1:i_sig*6,:);
                    DL_here = Ws(ii)*Linkage.DL((i_sig-1)*6+1:i_sig*6,:);
                    v_here  = norm(eta_here(4:6));
                    F = F-J_here'*DL_here*v_here*eta_here;
                end

                Qtemp = J_here'*M_here;
                M = M+Qtemp*J_here;
                F = F-Qtemp*psi_here-J_here'*dinamico_coadj(eta_here)*M_here*eta_here;
            end
            i_sig = i_sig+1;

        end

        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        psi_here  = Ad_gf_inv*psi_here;

        if ~Linkage.OneBasis
            dof_start = dof_start+dof_here;
        end
    end

    g_tip((i-1)*4+1:i*4,:)   = g_here;
    J_tip((i-1)*6+1:i*6,:)   = J_here;
    Jd_tip((i-1)*6+1:i*6,:)  = Jd_here;
    eta_tip((i-1)*6+1:i*6,:) = eta_here;
    psi_tip((i-1)*6+1:i*6,:) = psi_here;

end

%% Internal forces
if Linkage.Damped
    D = Linkage.D;
else
    D = 0;
end
K   = Linkage.K;
tau = -K*q - D*qd;

%% Actuation
if Linkage.Actuated

    nact   = Linkage.nact;
    B      = zeros(ndof,nact);
    N_jact = Linkage.N_jact;
    n_jact = Linkage.n_jact;
    Bj1    = Linkage.Bj1;
    na_j1  = size(Bj1,2);
    B(:,1:na_j1) = Linkage.Bj1;

    i_jact  = Linkage.i_jact;
    i_u     = na_j1+1;
    i_jactq = Linkage.i_jactq;

    for ii=na_j1+1:N_jact
        i         = i_jact(ii);
        dof_here  = Linkage.CVRods{i}(1).dof;
        us_here   = i_u:i_u+dof_here-1;
        dofs_here = i_jactq(us_here);
        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='C'
            B(dofs_here,us_here) = eye(2);
        else
            i_sig_here       = Linkage.ActuationPrecompute.i_sig_act(ii);
            if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
                gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
            else
                gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi{1};
            end
            AdgstepinvS_here = dinamico_Adjoint(gi)*J((i_sig_here-1)*6+1:i_sig_here*6,i_jactq(i_u:i_u+dof_here-1));
            Phi_here         = Linkage.CVRods{i}(1).Phi;
            B(dofs_here,us_here) = AdgstepinvS_here'*Phi_here;
        end
        i_u = i_u+dof_here;
    end

    if Linkage.n_sact>0
        dof_start = 1;
        for i=1:N
            dof_here = Linkage.CVRods{i}(1).dof;
            if ~Linkage.OneBasis
                dof_start = dof_start+dof_here;
            end
            for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
                na_here   = length(Linkage.i_sact{i}{j});
                dof_here  = Linkage.CVRods{i}(j+1).dof;
                dofs_here = dof_start:dof_start+dof_here-1;
                nip       = Linkage.CVRods{i}(j+1).nip;
                if ~Linkage.OneBasis
                    dof_start = dof_start+dof_here;
                end
                if na_here>0
                    Ws      = Linkage.CVRods{i}(j+1).Ws;
                    B_here  = zeros(dof_here,na_here);
                    for ii=1:nip
                        if Ws(ii)>0
                            Phi  = Linkage.CVRods{i}(j+1).Phi((ii-1)*6+1:ii*6,:);
                            xi   = Phi*q(dofs_here)+Linkage.CVRods{i}(j+1).xi_star((ii-1)*6+1:ii*6,1);
                            Phi_a = zeros(6,na_here);
                            xihat_123 = [0 -xi(3) xi(2) xi(4);xi(3) 0 -xi(1) xi(5);-xi(2) xi(1) 0 xi(6)];
                            ia_here = 1;
                            for ia=Linkage.i_sact{i}{j}
                                dc        = Linkage.dc{ia,i}{j}(:,ii);
                                dcp       = Linkage.dcp{ia,i}{j}(:,ii);
                                Phi_a(:,ia_here) = SoftActuator_FD(dc,dcp,xihat_123);
                                ia_here   = ia_here+1;
                            end
                            B_here = B_here+Ws(ii)*Phi'*Phi_a;
                        end
                    end
                    B(dofs_here,n_jact+Linkage.i_sact{i}{j}) = B_here;
                end
            end
        end
    end

    tau = tau + B*u;
end

%% Solve
qdd = M \ (tau + F);

end