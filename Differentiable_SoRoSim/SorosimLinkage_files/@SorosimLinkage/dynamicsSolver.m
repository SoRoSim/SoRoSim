%Function to compute [qdd_u (unknown); u_u; lambda] for given q,qd,[u_k,qdd_k]
%Single pass algorithm using D'Alembert-Kane method
%Last modified by Anup Teejo Mathew 21.01.2024
function [y,C,B_action,action,z_star,J_f] = dynamicsSolver(Linkage,t,qqd,action) %x is [qdd_u (unknown); u_u; lambda], action is [u_k,qdd_k]

%z_star is solution for each pair (6xncp), J_f is contact Jacobian (6*ncp,6)

ndof = Linkage.ndof;
N    = Linkage.N;
nsig = Linkage.nsig;

if Linkage.CAI
    action = CustomActuatorInput(Linkage,qqd,t); %x is qqd here
end

q  = qqd(1:Linkage.ndof);
qd = qqd(Linkage.ndof+1:2*Linkage.ndof);

M = zeros(ndof,ndof); %Generalized Mass Matrix
F = zeros(ndof,1); %External and Coriolis force

g   = zeros(4*nsig,4);
J   = zeros(6*nsig,ndof);
Jd  = zeros(6*nsig,ndof); % for dependent base Jd is Z

dof_start = 1; %starting dof of current piece
i_sig     = 1;

g_ini    = Linkage.g_ini; %initial configuration of all link wrt its previous link
g_tip   = repmat(eye(4),N,1);
J_tip   = repmat(zeros(6,ndof),N,1);
Jd_tip  = repmat(zeros(6,ndof),N,1); 
eta_tip = zeros(N*6,1); %total velocity J*qd
psi_tip = zeros(N*6,1); %total velocity Jd*qd
iLpre     = Linkage.iLpre;

for i=1:N

    G = Linkage.G;
    if ~Linkage.Gravity
        G = G*0;
    elseif Linkage.UnderWater %G is reduced by a factor of 1-rho_water/rho_body
        G=G*(1-Linkage.Rho_water/Linkage.VLinks(Linkage.LinkIndex(i)).Rho);
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

    %Joint
    %0 of joint

    g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
    J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
    Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
    i_sig                       = i_sig+1;

    dof_here = Linkage.CVRods{i}(1).dof;
    dofs_here = dof_start:dof_start+dof_here-1;
    q_here   = q(dofs_here);
    qd_here  = qd(dofs_here);
    Phi_here = Linkage.CVRods{i}(1).Phi;
    xi_star  = Linkage.CVRods{i}(1).xi_star;
    
    S_here  = zeros(6,ndof);
    Sd_here = zeros(6,ndof);
    if dof_here==0 %fixed joint (N)
        gstep   = eye(4);
    else
        xi          = Phi_here*q_here+xi_star;
        xid         = Phi_here*qd_here;

        [gstep,Tg,Tgd] = variable_expmap_gTgTgd_mex(xi,xid);

        S_here(:,dofs_here)  = Tg*Phi_here;
        Sd_here(:,dofs_here) = Tgd*Phi_here;
    end

    %updating g, Jacobian, Jacobian_dot and eta 0 to 1 of joint
    g_here         = g_here*gstep;
    Ad_g_joint_inv = dinamico_Adjoint(ginv(gstep));
    J_here         = Ad_g_joint_inv*(J_here+S_here); 
    Jd_here        = Ad_g_joint_inv*(Jd_here+Sd_here+dinamico_adj(eta_here)*S_here); %must use previous eta
    psi_here       = Ad_g_joint_inv*(psi_here+(Sd_here(:,dofs_here)+dinamico_adj(eta_here)*S_here(:,dofs_here))*qd_here); %must use previous eta
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
            M_here = M_here+Linkage.M_added((i_sig-1)*6+1:i_sig*6,:); %Including Added Mass
            DL_here = Linkage.DL((i_sig-1)*6+1:i_sig*6,:);
            v_here = norm(eta_here(4:6)); %linear speed
            F = F-J_here'*DL_here*v_here*eta_here; %drag lift force
        end
        i_sig    = i_sig+1;

        Qtemp = J_here'*M_here; % temporary variable to avoid repetetion 
        M = M+Qtemp*J_here;
        F = F-Qtemp*psi_here-J_here'*dinamico_coadj(eta_here)*M_here*eta_here; %Centripetal and Coriolis
        
        % bringing all quantities to the end of rigid link
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        psi_here  = Ad_gf_inv*psi_here;

    end

    if ~Linkage.OneBasis %thinking about POD basis of Abdulaziz
        dof_start = dof_start+dof_here;
    end

    for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1 %will run only if soft link

        dof_here   = Linkage.CVRods{i}(j+1).dof;
        dofs_here  = dof_start:dof_start+dof_here-1;
        q_here     = q(dofs_here);
        qd_here    = qd(dofs_here);
        
        gi      = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        ld      = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        xi_star = Linkage.CVRods{i}(j+1).xi_star;
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
        psi_here  = Ad_gi_inv*psi_here;

        g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
        J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
        Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
       
        ii = 1;
        if Ws(ii)>0
            M_here = Ws(ii)*Ms(6*(ii-1)+1:6*ii,:);
            F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;

            if Linkage.UnderWater
                M_here  = M_here+Ws(ii)*Linkage.M_added((i_sig-1)*6+1:i_sig*6,:); %Including Added Mass
                DL_here = Ws(ii)*Linkage.DL((i_sig-1)*6+1:i_sig*6,:);
                v_here = norm(eta_here(4:6)); %linear speed
                F = F-J_here'*DL_here*v_here*eta_here; %drag lift force
            end
    
            Qtemp = J_here'*M_here; % temporary variable to avoid repetetion 
            M = M+Qtemp*J_here;
            F = F-Qtemp*psi_here-J_here'*dinamico_coadj(eta_here)*M_here*eta_here; %Centripetal and Coriolis
        end
        i_sig    = i_sig+1;

        for ii=2:nip

            H = (Xs(ii)-Xs(ii-1))*ld;
            
            if Linkage.Z_order==4
                
                fZ4 = ((sqrt(3)*H^2)/12); %just a factor

                xi_Z1here = xi_star(6*(ii-2)+1:6*(ii-1),2); 
                xi_Z2here = xi_star(6*(ii-2)+1:6*(ii-1),3);    
                Phi_Z1here = Linkage.CVRods{i}(j+1).Phi_Z1(6*(ii-2)+1:6*(ii-1),:);%note this step
                Phi_Z2here = Linkage.CVRods{i}(j+1).Phi_Z2(6*(ii-2)+1:6*(ii-1),:);

                if dof_here>0

                    xi_Z1here = Phi_Z1here*q_here+xi_Z1here;
                    xi_Z2here = Phi_Z2here*q_here+xi_Z2here;

                    xid_Z1here  = Phi_Z1here*qd_here;
                    ad_xi_Z1here = dinamico_adj(xi_Z1here);

                    Z_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                              fZ4*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);

                    Zd_here = 2*fZ4*dinamico_adj(xid_Z1here)*Phi_Z2here; %not technically correct, but this term is always multiplied with qd
                    Omegad_here   = Z_here*qd_here;

                else

                    ad_xi_Z1here = dinamico_adj(xi_Z1here);
                    Z_here  = [];
                    Zd_here = []; 
                    Omegad_here = zeros(6,1); 

                end

                Omega_here = (H/2)*(xi_Z1here+xi_Z2here)+...
                             fZ4*ad_xi_Z1here*xi_Z2here;
                      
            else % order 2
                
                xi_Zhere = xi_star(6*(ii-2)+1:6*(ii-1),4);
                Phi_Zhere = Linkage.CVRods{i}(j+1).B_Z(6*(ii-2)+1:6*(ii-1),:);%note this step

                if dof_here>0
                    xi_Zhere = Phi_Zhere*q_here+xi_Zhere;
                    Z_here = H*Phi_Zhere;
                    Omegad_here = Z_here*qd_here;
                else
                    Z_here = H*Phi_Zhere;
                    Omegad_here   = zeros(6,1); 
                end
                Omega_here  = H*xi_Zhere;

            end

            [gstep,Tg,Tgd] = variable_expmap_gTgTgd_mex(Omega_here,Omegad_here); % mex code, C program

            S_here  = zeros(6,ndof);
            Sd_here = zeros(6,ndof);

            if dof_here>0
                S_here(:,dofs_here)  = Tg*Z_here;
                Sd_here(:,dofs_here) = Tgd*Z_here;
                if Linkage.Z_order==4
                    Sd_here(:,dofs_here) = Sd_here(:,dofs_here)+Tg*Zd_here;
                end
            end
            
            %updating g, Jacobian, Jacobian_dot and eta
            g_here     = g_here*gstep;
            Ad_gh_inv  = dinamico_Adjoint(ginv(gstep));
            J_here     = Ad_gh_inv*(J_here+S_here); %full
            Jd_here    = Ad_gh_inv*(Jd_here+Sd_here+dinamico_adj(eta_here)*S_here); %full
            psi_here   = Ad_gh_inv*(psi_here+(Sd_here(:,dofs_here)+dinamico_adj(eta_here)*S_here(:,dofs_here))*qd_here);
            eta_here   = Ad_gh_inv*(eta_here+S_here(:,dofs_here)*qd_here);
            
            g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
            J((i_sig-1)*6+1:i_sig*6,:)  = J_here;
            Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
            
            %integrals evaluation
            if Ws(ii)>0
                M_here = Ws(ii)*Ms(6*(ii-1)+1:6*ii,:);
                F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;
    
                if Linkage.UnderWater
                    M_here  = M_here+Ws(ii)*Linkage.M_added((i_sig-1)*6+1:i_sig*6,:); %Including Added Mass
                    DL_here = Ws(ii)*Linkage.DL((i_sig-1)*6+1:i_sig*6,:);
                    v_here = norm(eta_here(4:6)); %linear speed
                    F = F-J_here'*DL_here*v_here*eta_here; %drag lift force
                end
        
                Qtemp = J_here'*M_here; % temporary variable to avoid repetetion 
                M = M+Qtemp*J_here;
                F = F-Qtemp*psi_here-J_here'*dinamico_coadj(eta_here)*M_here*eta_here; %Centripetal and Coriolis
            end
            i_sig    = i_sig+1;

        end

        %updating g, Jacobian, Jacobian_dot and eta at X=L
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        psi_here  = Ad_gf_inv*psi_here;

        if ~Linkage.OneBasis %thinking about POD basis of Abdulaziz
            dof_start = dof_start+dof_here;
        end
    end
    
    g_tip((i-1)*4+1:i*4,:)   = g_here;
    J_tip((i-1)*6+1:i*6,:)   = J_here;
    Jd_tip((i-1)*6+1:i*6,:)  = Jd_here;
    eta_tip((i-1)*6+1:i*6,:) = eta_here;
    psi_tip((i-1)*6+1:i*6,:) = psi_here;

end

%% Contact Forces (make generic later)
z_star = zeros(6,Linkage.ncp);
J_f    = zeros(6*Linkage.ncp,6);

for icp = 1:Linkage.ncp
    i_sig = Linkage.Pairs(icp).body1.i_sig;
    g1 = g((i_sig-1)*4+1:i_sig*4,:);
    
    g2 = eye(4);

    out = Linkage.Pairs(icp).solveNarrowPhase(g1, g2);

    if out.alpha < 1

        Linkage.Pairs(icp).contact_active = true;
        z_star(:,icp) = [out.x; out.alpha; out.lambda1; out.lambda2];
        J_f(1+6*(icp-1):6*icp,:) = out.J;

        % relative transform (frame2 expressed in frame1)
        g_12 = Linkage.Pairs(icp).get_relative(g1, g2);
        r12  = g_12(1:3,4);                    % frame 1

        % penetration proxy (positive)
        delta = (1/out.alpha - 1) * norm(r12);
        fn = Linkage.penalty.k_n * delta^Linkage.penalty.e_n;

        % normal from body1 implicit gradient (frame 1)
        grad = Linkage.Pairs(icp).body1.get_grad(out.x);
        n = grad / norm(grad);

        % force on body1 in frame 1
        fc = - fn * n; %(repulsion)

        % wrench about body frame origin (frame 1)
        Fc = [dinamico_tilde(out.x)*fc; fc];

        % body Jacobian (frame 1)
        J1 = J((i_sig-1)*6+1:i_sig*6,:);
        F = F + J1' * Fc;
    end
end


%% Point Force

for ip=1:Linkage.np
    Fp_here = Linkage.Fp_vec{ip}(t);
    i_sig = Linkage.Fp_sig(ip);

    if ~Linkage.LocalWrench(ip) %if in the global frame
        g_here = g((i_sig-1)*4+1:i_sig*4,:);
        g_here(1:3,4) = zeros(3,1); %only rotational part
        Fp_here = (dinamico_Adjoint(g_here))'*Fp_here; %rotated into the local frame. Adj' = coAdj^-1
    end

    J_here = J((i_sig-1)*6+1:i_sig*6,:);
    F=F+J_here'*Fp_here; %point force
end
%% Closed Chain Constraint

A  = zeros(Linkage.CLprecompute.nCLp,ndof);
Ad = zeros(Linkage.CLprecompute.nCLp,ndof);
e  = zeros(Linkage.CLprecompute.nCLp,1);

il_start = 1; %starting index of current closed chain joint lambda

for iCLj=1:Linkage.nCLj
    
    i_sigA = Linkage.CLprecompute.i_sigA(iCLj);
    i_sigB = Linkage.CLprecompute.i_sigB(iCLj);
    Phi_p = Linkage.CLprecompute.Phi_p{iCLj};
    nl = size(Phi_p,2); %total number of constraint forces in the iCLj th closed chain joint
    
    if i_sigA>0
        %Linkageansformation from computational point to CLj
        if Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).linktype=='s'
            g_sigA2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).gf{end}*Linkage.gACLj{iCLj};
        else
            g_sigA2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).gf*Linkage.gACLj{iCLj};
        end
        gA = g((i_sigA-1)*4+1:i_sigA*4,:)*g_sigA2CLj;
        JA = dinamico_Adjoint(ginv(g_sigA2CLj))*J((i_sigA-1)*6+1:i_sigA*6,:);
        JdA = dinamico_Adjoint(ginv(g_sigA2CLj))*Jd((i_sigA-1)*6+1:i_sigA*6,:);
    else %if ground
        gA = Linkage.gACLj{iCLj};
        JA = zeros(6,ndof);
        JdA = zeros(6,ndof);
    end

    if i_sigB>0
        %Linkageansformation from computational point to CLj
        if Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).linktype=='s'
            g_sigB2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).gf{end}*Linkage.gBCLj{iCLj};
        else
            g_sigB2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).gf*Linkage.gBCLj{iCLj};
        end
        gB = g((i_sigB-1)*4+1:i_sigB*4,:)*g_sigB2CLj;
        JB = dinamico_Adjoint(ginv(g_sigB2CLj))*J((i_sigB-1)*6+1:i_sigB*6,:);
        JdB = dinamico_Adjoint(ginv(g_sigB2CLj))*Jd((i_sigB-1)*6+1:i_sigB*6,:);
    else %if ground
        gB = Linkage.gBCLj{iCLj};
        JB = zeros(6,ndof);
        JdB = zeros(6,ndof);
    end
    
    %constraint force is in the B frame. Hence we need to bring all to B frame
    gBA = ginv(gB)*gA;
    OmegaBA = piecewise_logmap(gBA); %twist vector from B to A in R^6
    Adj_gBA = dinamico_Adjoint(gBA);
    diffJAB = Adj_gBA*JA-JB; %in B frame
    diffJdAB = Adj_gBA*JdA-JdB; %in B frame

    A(il_start:il_start+nl-1,:) = Phi_p'*diffJAB;
    Ad(il_start:il_start+nl-1,:) = Phi_p'*(diffJdAB+dinamico_adj(diffJAB*qd)*Adj_gBA*JA);
    e(il_start:il_start+nl-1) = Phi_p'*OmegaBA;
    
    il_start = il_start+nl;
end

%A should be FULL ROW RANK Matrix. Otherwise Problem!

% A possible fix below (needs more work)
% if rank(A)<size(A,1) %Have to take care of this
%     [~,iRows] = LIRows(A); 
%     e         = e(iRows); %iRows are the linearly independent rows of A. Comment this for some
% end

%% Custom External Force

if Linkage.CEF
    Fext  = CustomExtForce(Linkage,q,g,J,t,qd,Jd); %should be point wrench in local frame and its derivatives
    for i_sig=1:Linkage.nsig
        F = F+J((i_sig-1)*6+1:i_sig*6,:)'*Fext;
    end
end

%% Internal force (Stiffness, Damping and Actuation)

if Linkage.Damped
    D = Linkage.D;
else
    D = 0;
end
K = Linkage.K; 
tau = -K*q-D*qd; %tau = Bu-Kq

if Linkage.Actuated
    
    nact = Linkage.nact;
    B = zeros(ndof,nact); %Arranged like: [Bj1 (1D) joints, Bjm (multi) joints, Bsoft]

    N_jact       = Linkage.N_jact; %number of Links whos joints are actuated
    n_jact       = Linkage.n_jact; %total number of joint actutators (3 for spherical joint)
    %revolute, prismatic, helical joints (1D joints)
    Bj1          = Linkage.Bj1; %Generalized Actuation Basis of 1D joints
    na_j1        = size(Bj1,2); 
    B(:,1:na_j1) = Linkage.Bj1;

    %for other joints
    i_jact  = Linkage.i_jact; %indices of Links whos joints are actuated
    i_u     = na_j1+1; %index of columns and us
    i_jactq = Linkage.i_jactq; %indices of actutated qs (for rigid joint)
    
    for ii=na_j1+1:N_jact %Links with actuated multi DOF joints

        i = i_jact(ii); %corresponding Link number
        dof_here = Linkage.CVRods{i}(1).dof;
        us_here = i_u:i_u+dof_here-1;
        dofs_here = i_jactq(us_here);
        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='C' %cylindrical
            B(dofs_here,us_here) = eye(2);
        else %for multijoint it is complex refer to paper
            i_sig = Linkage.ActuationPrecompute.i_sig_act(ii); %at CM of rigid body or X=0 of first soft division
            
            if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
                gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
            else
                gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi{1};
            end

            AdgstepinvS_here = dinamico_Adjoint(gi)*J((i_sig-1)*6+1:i_sig*6,i_jactq(i_u:i_u+dof_here-1)); %at X=1 of the joint
            Phi_here = Linkage.CVRods{i}(1).Phi;
            B(dofs_here,us_here) = AdgstepinvS_here'*Phi_here;
        end
        i_u = i_u+dof_here;
    end

    %cable actuation %combine with FwdKinematics
    if Linkage.n_sact>0
        dof_start = 1;
        for i = 1:N
            dof_here = Linkage.CVRods{i}(1).dof;
            if ~Linkage.OneBasis
                dof_start=dof_start+dof_here;
            end
            for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
                na_here = length(Linkage.i_sact{i}{j});
                dof_here = Linkage.CVRods{i}(j+1).dof;
                dofs_here = dof_start:dof_start+dof_here-1;
                if ~Linkage.OneBasis
                    dof_start=dof_start+dof_here;
                end
                if na_here>0
                    Ws = Linkage.CVRods{i}(j+1).Ws;
                    B_here = zeros(dof_here,na_here);
                    for ii=1:nip
                        if Ws(ii)>0
                            Phi = Linkage.CVRods{i}(j+1).Phi((ii-1)*6+1:ii*6,:);
                            xi  = Phi*q(dofs_here)+Linkage.CVRods{i}(j+1).xi_star((ii-1)*6+1:ii*6,1);
                            Phi_a = zeros(6,na_here);
                            xihat_123  = [0 -xi(3) xi(2) xi(4);xi(3) 0 -xi(1) xi(5);-xi(2) xi(1) 0 xi(6)];%4th row is avoided to speedup calculation
                            ia_here = 1;
                            for ia =Linkage.i_sact{i}{j}
                                dc = Linkage.dc{ia,i}{j}(:,ii);
                                dcp = Linkage.dcp{ia,i}{j}(:,ii);
                                Phi_a(:,ia_here) = SoftActuator_FD(dc,dcp,xihat_123);
                                ia_here = ia_here+1;
                            end
                            B_here = B_here+Ws(ii)*Phi'*Phi_a;
                        end
                    end
                    B(dofs_here,n_jact+Linkage.i_sact{i}{j}) = B_here;
                end
            end
        end
    end
end

%% Custom Soft Actuation

if Linkage.CA
    Fact = CustomActuation(Linkage,q,g,J,t,qd,Jd);
    tauC = CustomActuation_Qspace(Linkage,Fact,dFact_dq);
    tau = tau+tauC;
end

%% solving for y

if Linkage.nCLj>0 %if there are closed chain joints

    nCLp = Linkage.CLprecompute.nCLp;
    T_BS = Linkage.T_BS;

    if Linkage.Actuated
        B_action = [B(:,Linkage.ActuationPrecompute.index_u_k), -M(:,Linkage.ActuationPrecompute.index_q_k);zeros(nCLp,Linkage.nact-Linkage.ActuationPrecompute.n_k), -A(:,Linkage.ActuationPrecompute.index_q_k)];
        C = [M(:,Linkage.ActuationPrecompute.index_q_u), -B(:,Linkage.ActuationPrecompute.index_u_u), -A';A(:,Linkage.ActuationPrecompute.index_q_u), zeros(nCLp,Linkage.ActuationPrecompute.n_k+nCLp)];
        b = B_action*action+[tau+F;-Ad*qd-(2/T_BS)*A*qd-(1/T_BS^2)*e];
    else
        C = [M, -A';A, zeros(nCLp,nCLp)];
        B_action = [];
        b = [tau+F;-Ad*qd-(2/T_BS)*A*qd-(1/T_BS^2)*e];
    end
else

    if Linkage.Actuated
        B_action = [B(:,Linkage.ActuationPrecompute.index_u_k), -M(:,Linkage.ActuationPrecompute.index_q_k)];
        C = [M(:,Linkage.ActuationPrecompute.index_q_u), -B(:,Linkage.ActuationPrecompute.index_u_u)];
        b = B_action*action+tau+F;
    else
        C = M;
        B_action = [];
        b = tau+F;
    end

end

y = C\b;

end