%Function that calculates the generalized coriolis matrix  at every significant points (24.05.2021)

function C = GeneralizedCoriolisMatrix(Linkage,q,qd)

if isrow(q)
    q=q';
end
if isrow(qd)
    qd=qd';
end

N         = Linkage.N;
ndof      = Linkage.ndof;
g_ini     = Linkage.g_ini; %initial configuration of all link wrt its previous link
iLpre     = Linkage.iLpre;
g_Ltip    = repmat(eye(4),N,1);
J_Ltip    = repmat(zeros(6,ndof),N,1);
Jd_Ltip   = repmat(zeros(6,ndof),N,1);
eta_Ltip  = zeros(N*6,1); %total velocity J*qd+eta_dBdt
C         = zeros(ndof,ndof);
dof_start = 1; %starting dof of current piece
i_sig_nj  = 1;

for i = 1:N
    
    if iLpre(i)>0
        g_here       = g_Ltip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        Ad_g_ini_inv = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)));
        J_here       = Ad_g_ini_inv*J_Ltip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        Jd_here      = Ad_g_ini_inv*Jd_Ltip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        eta_here     = Ad_g_ini_inv*eta_Ltip((iLpre(i)-1)*6+1:iLpre(i)*6);
    else
        g_here   = g_ini((i-1)*4+1:i*4,:);
        J_here   = zeros(6,ndof);
        Jd_here  = zeros(6,ndof);
        eta_here = zeros(6,1);
    end
    
    %Joint
    dof_here = Linkage.CVRods{i}(1).dof;
    q_here   = q(dof_start:dof_start+dof_here-1);
    qd_here  = qd(dof_start:dof_start+dof_here-1);
    Phi_here   = Linkage.CVRods{i}(1).Phi;
    xi_star  = Linkage.CVRods{i}(1).xi_star;
    
    if dof_here == 0 %fixed joint (N)
        g_joint   = eye(4);
        S_here  = zeros(6,ndof);
        Sd_here = zeros(6,ndof);
    else
        xi               = Phi_here*q_here+xi_star;
        xid              = Phi_here*qd_here;
        [g_joint,Tg,Tgd] = variable_expmap_gTgTgd_mex(xi,xid);

        S_here                                    = zeros(6,ndof);
        S_here(:,dof_start:dof_start+dof_here-1)  = Tg*Phi_here;
        Sd_here                                   = zeros(6,ndof);
        Sd_here(:,dof_start:dof_start+dof_here-1) = dinamico_adj(eta_here)*Tg*Phi_here+Tgd*Phi_here;
    end
    
    %updating g, Jacobian, Jacobian_dot and eta
    g_here         = g_here*g_joint;
    Ad_g_joint_inv = dinamico_Adjoint(ginv(g_joint));
    J_here         = Ad_g_joint_inv*(J_here+S_here);
    Jd_here        = Ad_g_joint_inv*(Jd_here+Sd_here);
    eta_here       = Ad_g_joint_inv*(eta_here+S_here(:,dof_start:dof_start+dof_here-1)*qd_here);

    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype == 'r'
        
        gi        = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
        g_here    = g_here*gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        J_here    = Ad_gi_inv*J_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        
        M_here     = Linkage.VLinks(Linkage.LinkIndex(i)).M;
        if ~isempty(Linkage.M_added)
            M_here = M_here+Linkage.M_added((i_sig_nj-1)*6+1:i_sig_nj*6,:); 
        end
        C          = C+J_here'*(M_here*Jd_here+dinamico_coadj(eta_here)*M_here*J_here);
        i_sig_nj   = i_sig_nj+1;
        
        % bringing all quantities to the end of rigid link
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
    end
    
    dof_start = dof_start+dof_here;
    
    for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1 %will run only if soft link
        
        dof_here = Linkage.CVRods{i}(j+1).dof;
        q_here   = q(dof_start:dof_start+dof_here-1);
        qd_here  = qd(dof_start:dof_start+dof_here-1);
        
        xi_star = Linkage.CVRods{i}(j+1).xi_star;
        gi      = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        lpf     = Linkage.VLinks(Linkage.LinkIndex(i)).lp{j};
        Ms      = Linkage.CVRods{i}(j+1).Ms;
        Xs      = Linkage.CVRods{i}(j+1).Xs;
        Ws      = Linkage.CVRods{i}(j+1).Ws;
        nip     = Linkage.CVRods{i}(j+1).nip;
        
        %updating g, Jacobian, Jacobian_dot and eta at X=0
        g_here    = g_here*gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        J_here    = Ad_gi_inv*J_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        
        %scaling of quantities
        Lscale         = lpf;
        g_here(1:3,4)  = g_here(1:3,4)/Lscale;
        J_here(4:6,:)  = J_here(4:6,:)/Lscale;
        Jd_here(4:6,:) = Jd_here(4:6,:)/Lscale;
        eta_here(4:6)  = eta_here(4:6)/Lscale;
        
        ii = 1;
        if Ws(ii)>0
            W_here   =Ws(ii);
            Ms_here = Ms(6*(ii-1)+1:6*ii,:);
            if isempty(Linkage.M_added)
                Ms_here_add = Ms_here;
            else
                Ms_here_add = Ms_here+Linkage.M_added((i_sig_nj-1)*6+1:i_sig_nj*6,:);
            end
            Ms_here_add(1:3,:) = Ms_here_add(1:3,:)/Lscale;
            Ms_here_add(4:6,:) = Ms_here_add(4:6,:)*Lscale;
            C     = C+(W_here*J_here'*Ms_here_add*Jd_here+W_here*J_here'*dinamico_coadj(eta_here)*Ms_here_add*J_here)*Lscale^2;
        end
        
        i_sig_nj = i_sig_nj+1;
        for ii = 2:nip
            
            H    = Xs(ii)-Xs(ii-1);
            
            if Linkage.Z_order==4
                
                xi_Z1here = xi_star(6*(ii-2)+1:6*(ii-1),2); 
                xi_Z2here = xi_star(6*(ii-2)+1:6*(ii-1),3);
                xi_Z1here(1:3) = xi_Z1here(1:3)*Lscale; %scaling
                xi_Z2here(1:3) = xi_Z2here(1:3)*Lscale;
                
                %B is Phi for independent basis and Bq is Psi for dependent basis
                    
                Phi_Z1here  = Linkage.CVRods{i}(j+1).Phi_Z1(6*(ii-2)+1:6*(ii-1),:);%note this step
                Phi_Z2here  = Linkage.CVRods{i}(j+1).Phi_Z2(6*(ii-2)+1:6*(ii-1),:);

                if dof_here>0

                    xi_Z1here = Phi_Z1here*q_here+xi_Z1here;
                    xi_Z2here = Phi_Z2here*q_here+xi_Z2here;

                    xid_Z1here  = Phi_Z1here*qd_here;

                    ad_xi_Z1here = dinamico_adj(xi_Z1here);

                    PhiGamma_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                                       ((sqrt(3)*H^2)/12)*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);

                    Gammadd_Z4_dq_here = ((sqrt(3)*H^2)/6)*dinamico_adj(xid_Z1here)*Phi_Z2here; 

                    Gammad_here   = PhiGamma_here*qd_here;

                else

                    ad_xi_Z1here = dinamico_adj(xi_Z1here);
                    PhiGamma_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                                       ((sqrt(3)*H^2)/12)*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);
                    Gammadd_Z4_dq_here = zeros(6,dof_here); 
                    Gammad_here   = zeros(6,1); 

                end

                Gamma_here = (H/2)*(xi_Z1here+xi_Z2here)+...
                             ((sqrt(3)*H^2)/12)*ad_xi_Z1here*xi_Z2here;
                      
            else % order 2
                
                xi_Zhere = xi_star(6*(ii-2)+1:6*(ii-1),4);
                xi_Zhere(1:3) = xi_Zhere(1:3)*Lscale; %scaling

                Phi_Zhere  = Linkage.CVRods{i}(j+1).Phi_Z(6*(ii-2)+1:6*(ii-1),:);%note this step

                if dof_here>0
                    xi_Zhere = Phi_Zhere*q_here+xi_Zhere;

                    PhiGamma_here = H*Phi_Zhere;

                    Gammad_here     = PhiGamma_here*qd_here;
                else
                    PhiGamma_here = H*Phi_Zhere;
                    Gammad_here   = zeros(6,1); 
                end

                Gamma_here  = H*xi_Zhere;

            end

            [gh,TGamma_here,TGammad_here] = variable_expmap_gTgTgd_mex(Gamma_here,Gammad_here); % mex code, C program

            S_here                                    = zeros(6,ndof);
            S_here(:,dof_start:dof_start+dof_here-1)  = TGamma_here*PhiGamma_here;
            Sd_here                                   = zeros(6,ndof);
            Sd_here(:,dof_start:dof_start+dof_here-1) = dinamico_adj(eta_here)*S_here(:,dof_start:dof_start+dof_here-1)+TGammad_here*PhiGamma_here;
            
            if Linkage.Z_order==4
                Sd_here(:,dof_start:dof_start+dof_here-1) = Sd_here(:,dof_start:dof_start+dof_here-1)+TGamma_here*Gammadd_Z4_dq_here;
            end
            
            %updating g, Jacobian, Jacobian_dot and eta
            g_here     = g_here*gh;
            Ad_gh_inv  = dinamico_Adjoint(ginv(gh));
            J_here     = Ad_gh_inv*(J_here+S_here); %full
            Jd_here    = Ad_gh_inv*(Jd_here+Sd_here); %full
            eta_here   = Ad_gh_inv*(eta_here+S_here(:,dof_start:dof_start+dof_here-1)*qd_here);

            
            %integrals evaluation
            if Ws(ii)>0
                W_here   =Ws(ii);
                Ms_here = Ms(6*(ii-1)+1:6*ii,:);
                if isempty(Linkage.M_added)
                    Ms_here_add = Ms_here;
                else
                    Ms_here_add = Ms_here+Linkage.M_added((i_sig_nj-1)*6+1:i_sig_nj*6,:);
                end
                Ms_here_add(1:3,:) = Ms_here_add(1:3,:)/Lscale;
                Ms_here_add(4:6,:) = Ms_here_add(4:6,:)*Lscale;
                C     = C+(W_here*J_here'*Ms_here_add*Jd_here+W_here*J_here'*dinamico_coadj(eta_here)*Ms_here_add*J_here)*Lscale^2;
            end
            i_sig_nj = i_sig_nj+1;
        end
        %scaling back quantities
        g_here(1:3,4)  = g_here(1:3,4)*Lscale;
        J_here(4:6,:)  = J_here(4:6,:)*Lscale;
        Jd_here(4:6,:) = Jd_here(4:6,:)*Lscale;
        eta_here(4:6)  = eta_here(4:6)*Lscale;
        
        %updating g, Jacobian, Jacobian_dot and eta at X=L
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        
        dof_start  = dof_start+dof_here;
    end
    g_Ltip((i-1)*4+1:i*4,:)  = g_here;
    J_Ltip((i-1)*6+1:i*6,:)  = J_here;
    Jd_Ltip((i-1)*6+1:i*6,:) = Jd_here;
    eta_Ltip((i-1)*6+1:i*6,:) = eta_here;
end
end

