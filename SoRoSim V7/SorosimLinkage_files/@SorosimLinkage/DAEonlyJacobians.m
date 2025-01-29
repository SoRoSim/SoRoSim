function [dID_dq,dID_dqd,dID_dqdd,dID_dlambda,dtau_dq,dtau_dqd,dtau_du,de_dq,de_dqd,de_dqdd,daction_dq,daction_dqd] = DAEonlyJacobians(Linkage,q,qd,qdd,u,lambda,index)

%For index 1 DAE e=e(q,qd,qdd). For index 3 DAE e=e(q)

N    = Linkage.N; %Total number of Links (including repetition)
ndof = Linkage.ndof; %Total dof of the Linkage
nsig = Linkage.nsig; %Total number of computational points
nj   = Linkage.nj; %Total number of joints (rigid and virtual soft joints (nip-1))
nact = Linkage.nact;

if Linkage.CAI
    [~,daction_dx] = CustomActuatorInput(Linkage,[q;qd]); %x is qqd
    daction_dq  = daction_dx(:,1:ndof);
    daction_dqd = daction_dx(:,ndof+1:2*ndof);
else
    daction_dq  = 0; %0 is better than zeros(nact,ndof)
    daction_dqd = 0;
end

dID_dq   = zeros(ndof,ndof);
dID_dqd  = zeros(ndof,ndof);
dID_dqdd = zeros(ndof,ndof); %same as M
% dID_dlambda = -A'

%dtau_dq, dtau_dqd, dtau_du (B) are initialized later

if Linkage.nCLj>1
    nCLp  = Linkage.CLprecompute.nCLp;
    de_dq = zeros(nCLp,ndof);
    if index == 1
        de_dqd  = zeros(nCLp,ndof);
        % de_dqdd = A
    else
        de_dqd  = 0;
        de_dqdd = 0;
    end
else
    de_dq = []; de_dqd = []; de_dqdd = [];
end

Fk = zeros(6*nsig,1); %External point force at every significant point

%% Forward kinematic pass: evaluate kinematic, and derivatives terms at every virtual joint and significant points

h = zeros(1,nj);            %Joint length. h=1 for rigid joints
Omega = zeros(6,nj);        %Joint twist
Z = zeros(6*nj,ndof);       %Joint basis. Z=Phi for rigid joints (sparse matrix, can we avoid?)
gstep = zeros(4*nj,4);      %Linkageansformation from X_alpha to X_alpha+1 (0 to 1 for rigid joints)
Adgstepinv = zeros(6*nj,6); %As the name says!
T = zeros(6*nj,6);          %Tangent vector at each joint (T(Omega))

S = zeros(6*nj,ndof);         %S is joint motion subspace (sparse matrix for a reason)
Sd = zeros(6*nj,ndof);        %Sd is time derivative of joint motion subspace (sparse matrix for a reason)
dS_dq_qd = zeros(6*nj,ndof);  %as the name says (sparse matrix for a reason)
dS_dq_qdd = zeros(6*nj,ndof); %as the name says (sparse matrix for a reason)
dSd_dq_qd = zeros(6*nj,ndof); %as the name says (sparse matrix for a reason)

R = zeros(6*nj,ndof); %relative joint velocity gradient wrt q (sparse matrix for a reason)
L = zeros(6*nj,ndof); %one part of relative joint acceleration gradient wrt q (sparse matrix for a reason)
Q = zeros(6*nj,ndof); %L - gravity gravity derivative wrt q at joint (sparse matrix for a reason)
Y = zeros(6*nj,ndof); %Y is  R+Sd+ad_eta*S

f = zeros(4,nj);            %function of theta, required for the computations of Tangent operator
fd = zeros(4,nj);           %required for the derivative of Tangent operator
adjOmegap = zeros(24*nj,6); %Powers of adjOmega (1-4), used later

g = zeros(4*nsig,4);    %Fwd kinematics

J    = zeros(6*nsig,ndof); %Jacobian (J is S_B)
eta  = zeros(6*nsig,1);    %velocity twist
Jd   = zeros(6*nsig,ndof); %Jacobian dot (Jd is Sd_B)
etad = zeros(6*nsig,1);    %acceleration twist

R_B = zeros(6*nsig,ndof); %Joint velocity gradient wrt q
L_B = zeros(6*nsig,ndof); %one part of joint acceleration gradient wrt q. deta_dot/dq = L_B-ad(eta)*R_B
Q_B = zeros(6*nsig,ndof); %L_B - gravity gradient wrt q
Y_B = zeros(6*nsig,ndof); %Joint acceleration gradient wrt qd, Y_B = Jd+R_B

%For branched chain computation
g_tip = repmat(eye(4),N,1);

J_tip    = zeros(N*6,ndof);
eta_tip  = zeros(N*6,1);
Jd_tip   = zeros(N*6,ndof);
etad_tip = zeros(N*6,1);

R_Btip = zeros(N*6,ndof);
L_Btip = zeros(N*6,ndof);
Q_Btip = zeros(N*6,ndof);

dof_start = 1; %starting dof of current rod
i_sig = 1;     %current computational point index
ij = 1;        %current virtual joint index

iLpre = Linkage.iLpre;
g_ini = Linkage.g_ini;

for i=1:N

    G = Linkage.G;
    if ~Linkage.Gravity
        G = G*0;
    elseif Linkage.UnderWater %G is reduced by a factor of 1-rho_water/rho_body
        G = G*(1-Linkage.Rho_water/Linkage.VLinks(Linkage.LinkIndex(i)).Rho);
    end

    if iLpre(i)>0
        g_here = g_tip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        Ad_g_ini_inv = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)));

        J_here    = Ad_g_ini_inv*J_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        eta_here  = Ad_g_ini_inv*eta_tip((iLpre(i)-1)*6+1:iLpre(i)*6);
        Jd_here   = Ad_g_ini_inv*Jd_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        etad_here = Ad_g_ini_inv*etad_tip((iLpre(i)-1)*6+1:iLpre(i)*6);

        R_Bhere = Ad_g_ini_inv*R_Btip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        L_Bhere = Ad_g_ini_inv*L_Btip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        Q_Bhere = Ad_g_ini_inv*Q_Btip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
    else
        g_here = g_ini((i-1)*4+1:i*4,:);

        J_here    = zeros(6,ndof);
        eta_here  = zeros(6,1);
        Jd_here   = zeros(6,ndof);
        etad_here = zeros(6,1);

        R_Bhere = zeros(6,ndof);
        L_Bhere = zeros(6,ndof);
        Q_Bhere = zeros(6,ndof);
    end

    %Joint

    %0 of joint
    g((i_sig-1)*4+1:4*i_sig,:) = g_here;

    J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
    eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
    Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
    etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;

    L_B((i_sig-1)*6+1:6*i_sig,:) = L_Bhere;
    R_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere;
    Q_B((i_sig-1)*6+1:6*i_sig,:) = Q_Bhere;
    Y_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere+Jd_here;

    dof_here  = Linkage.CVRods{i}(1).dof;
    dofs_here = dof_start:dof_start+dof_here-1;

    Phi_here = Linkage.CVRods{i}(1).Phi;
    xi_star  = Linkage.CVRods{i}(1).xi_star;
    h(ij) = 1;

    %computation of kinematic and differential kinematic quantities at the rigid joint
    %[Omega,Z,g,T,S,Sd,f,fd,adjOmegap,dSdq_qd,dSdq_qdd,dSddq_qd]=RigidJointDifferentialKinematics(Phi,xi_star,q,qd,qdd)
    [Omega(:,ij),Z((ij-1)*6+1:6*ij,dofs_here),gstep((ij-1)*4+1:4*ij,:),T((ij-1)*6+1:6*ij,:),S((ij-1)*6+1:6*ij,dofs_here),...
     Sd((ij-1)*6+1:6*ij,dofs_here),f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),dS_dq_qd((ij-1)*6+1:6*ij,dofs_here),...
     dS_dq_qdd((ij-1)*6+1:6*ij,dofs_here),dSd_dq_qd((ij-1)*6+1:6*ij,dofs_here)] = RigidJointDifferentialKinematics_mex(Phi_here,xi_star,q(dofs_here),qd(dofs_here),qdd(dofs_here));
    Adgstepinv((ij-1)*6+1:6*ij,:) = dinamico_Adjoint(ginv(gstep((ij-1)*4+1:4*ij,:)));

    eta_plus_here  = eta_here+S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here); %eta of X=1 expressed in the frame of X=0
    etad_plus_here = etad_here+S((ij-1)*6+1:6*ij,dofs_here)*qdd(dofs_here)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here)+Sd((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here);
    
    R((ij-1)*6+1:6*ij,dofs_here) = dinamico_adj(eta_plus_here)*S((ij-1)*6+1:6*ij,dofs_here)+dS_dq_qd((ij-1)*6+1:6*ij,dofs_here);
    L((ij-1)*6+1:6*ij,dofs_here) = dinamico_adj(etad_plus_here)*S((ij-1)*6+1:6*ij,dofs_here)+dinamico_adj(eta_plus_here)*R((ij-1)*6+1:6*ij,dofs_here)...
                                  +dinamico_adj(eta_here)*dS_dq_qd((ij-1)*6+1:6*ij,dofs_here)+dSd_dq_qd((ij-1)*6+1:6*ij,dofs_here)+dS_dq_qdd((ij-1)*6+1:6*ij,dofs_here); % L is Q without gravity
    Q((ij-1)*6+1:6*ij,dofs_here) = L((ij-1)*6+1:6*ij,dofs_here)-dinamico_adj(dinamico_Adjoint(ginv(g_here))*G)*S((ij-1)*6+1:6*ij,dofs_here);
    Y((ij-1)*6+1:6*ij,dofs_here) = R((ij-1)*6+1:6*ij,dofs_here)+Sd((ij-1)*6+1:6*ij,:)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here);
    
    % from 0 to 1 of rigid joint
    g_here  = g_here*gstep((ij-1)*4+1:4*ij,:);

    J_here    = Adgstepinv((ij-1)*6+1:6*ij,:)*(J_here+S((ij-1)*6+1:6*ij,:));
    eta_here  = Adgstepinv((ij-1)*6+1:6*ij,:)*(eta_plus_here);
    Jd_here   = Adgstepinv((ij-1)*6+1:6*ij,:)*(Jd_here+Sd((ij-1)*6+1:6*ij,:)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here));
    etad_here = Adgstepinv((ij-1)*6+1:6*ij,:)*(etad_plus_here);

    R_Bhere = Adgstepinv((ij-1)*6+1:6*ij,:)*(R_Bhere+R((ij-1)*6+1:6*ij,:));
    L_Bhere = Adgstepinv((ij-1)*6+1:6*ij,:)*(L_Bhere+L((ij-1)*6+1:6*ij,:));
    Q_Bhere = Adgstepinv((ij-1)*6+1:6*ij,:)*(Q_Bhere+Q((ij-1)*6+1:6*ij,:));
    %Y_Bhere = R_Bhere+Jd_here;
    
    ij = ij+1;
    i_sig = i_sig+1;
    if ~Linkage.OneBasis %thinking about POD basis of Abdulaziz
        dof_start = dof_start+dof_here;
    end

    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
        %joint to CM
        gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        
        g_here  = g_here*gi;

        J_here    = Ad_gi_inv*J_here;
        eta_here  = Ad_gi_inv*eta_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        etad_here = Ad_gi_inv*etad_here;
    
        R_Bhere = Ad_gi_inv*R_Bhere;
        L_Bhere = Ad_gi_inv*L_Bhere;
        Q_Bhere = Ad_gi_inv*Q_Bhere;
        

        %CM is a significant point
        g((i_sig-1)*4+1:4*i_sig,:) = g_here;

        J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
        eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
        Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
        etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;
    
        L_B((i_sig-1)*6+1:6*i_sig,:) = L_Bhere;
        R_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere;
        Q_B((i_sig-1)*6+1:6*i_sig,:) = Q_Bhere;
        Y_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere+Jd_here;

        i_sig = i_sig+1;

        %CM to tip
        gf = Linkage.VLinks(Linkage.LinkIndex(i)).gf; 
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));

        g_here  = g_here*gf;

        J_here    = Ad_gf_inv*J_here;
        eta_here  = Ad_gf_inv*eta_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        etad_here = Ad_gf_inv*etad_here;
    
        R_Bhere = Ad_gf_inv*R_Bhere;
        L_Bhere = Ad_gf_inv*L_Bhere;
        Q_Bhere = Ad_gf_inv*Q_Bhere;
    end

    for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1 %will run only if npie>1, ie for soft links

        dof_here  = Linkage.CVRods{i}(j+1).dof;
        dofs_here = dof_start:dof_start+dof_here-1;
        
        ld = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        xi_star = Linkage.CVRods{i}(j+1).xi_star;
        Xs      = Linkage.CVRods{i}(j+1).Xs;
        nip     = Linkage.CVRods{i}(j+1).nip;
        h(ij:ij+nip-2) = (Xs(2:end)-Xs(1:end-1))*ld;
        
        gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j}; %X=1 of joint to first quadrature point at X=0 of the rod
        Ad_gi_inv = dinamico_Adjoint(ginv(gi)); %transformation from X=1 of joint or X=Ld of previous division to the X=0 of next division
        
        g_here  = g_here*gi;

        J_here    = Ad_gi_inv*J_here;
        eta_here  = Ad_gi_inv*eta_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        etad_here = Ad_gi_inv*etad_here;
    
        R_Bhere = Ad_gi_inv*R_Bhere;
        L_Bhere = Ad_gi_inv*L_Bhere;
        Q_Bhere = Ad_gi_inv*Q_Bhere;

        g((i_sig-1)*4+1:4*i_sig,:) = g_here;

        J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
        eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
        Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
        etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;
    
        L_B((i_sig-1)*6+1:6*i_sig,:) = L_Bhere;
        R_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere;
        Q_B((i_sig-1)*6+1:6*i_sig,:) = Q_Bhere;
        Y_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere+Jd_here;

        i_sig  = i_sig+1;

        for ii=1:nip-1 
            
            if Linkage.Z_order==4 %Zannah Quadrature order 4
                xi_star_Z1 = xi_star(6*(ii-1)+1:6*ii,2);
                xi_star_Z2 = xi_star(6*(ii-1)+1:6*ii,3);
                Phi_Z1 = Linkage.CVRods{i}(j+1).Phi_Z1(6*(ii-1)+1:6*ii,:);
                Phi_Z2 = Linkage.CVRods{i}(j+1).Phi_Z2(6*(ii-1)+1:6*ii,:);

                %[Omega,Z,g,T,S,Sd,f,fd,adjOmegap,dSdq_qd,dSdq_qdd,dSddq_qd]=SoftJointDifferentialKinematics_Z4(h,Phi_Z1,Phi_Z2,xi_star_Z1,xi_star_Z2,q,qd,qdd)
                [Omega(:,ij),Z((ij-1)*6+1:6*ij,dofs_here),gstep((ij-1)*4+1:4*ij,:),T((ij-1)*6+1:6*ij,:),S((ij-1)*6+1:6*ij,dofs_here),...
                 Sd((ij-1)*6+1:6*ij,dofs_here),f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),dS_dq_qd((ij-1)*6+1:6*ij,dofs_here),...
                 dS_dq_qdd((ij-1)*6+1:6*ij,dofs_here),dSd_dq_qd((ij-1)*6+1:6*ij,dofs_here)] = SoftJointDifferentialKinematics_Z4_mex(h(ij),Phi_Z1,Phi_Z2,xi_star_Z1,xi_star_Z2,q(dofs_here),qd(dofs_here),qdd(dofs_here));
                Adgstepinv((ij-1)*6+1:6*ij,:) = dinamico_Adjoint(ginv(gstep((ij-1)*4+1:4*ij,:)));
            else % order 2
                xi_star_Z  = xi_star(6*(ii-1)+1:6*ii,4);
                Phi_Z = Linkage.CVRods{i}(j+1).Phi_Z(6*(ii-1)+1:6*ii,:);
                
                %[Omega,Z,g,T,S,Sd,f,fd,adjOmegap,dSdq_qd,dSdq_qdd,dSddq_qd]=SoftJointDifferentialKinematics_Z2(h,Phi_Z,xi_star_Z,q,qd,qdd)
                [Omega(:,ij),Z((ij-1)*6+1:6*ij,dofs_here),gstep((ij-1)*4+1:4*ij,:),T((ij-1)*6+1:6*ij,:),S((ij-1)*6+1:6*ij,dofs_here),...
                 Sd((ij-1)*6+1:6*ij,dofs_here),f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),dS_dq_qd((ij-1)*6+1:6*ij,dofs_here),...
                 dS_dq_qdd((ij-1)*6+1:6*ij,dofs_here),dSd_dq_qd((ij-1)*6+1:6*ij,dofs_here)] = SoftJointDifferentialKinematics_Z2_mex(h(ij),Phi_Z,xi_star_Z,q(dofs_here),qd(dofs_here),qdd(dofs_here));
                Adgstepinv((ij-1)*6+1:6*ij,:) = dinamico_Adjoint(ginv(gstep((ij-1)*4+1:4*ij,:)));
            end
           
            eta_plus_here  = eta_here+S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here); %eta of next computational point expressed in the frame of current
            etad_plus_here = etad_here+S((ij-1)*6+1:6*ij,dofs_here)*qdd(dofs_here)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here)+Sd((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here);
            
            R((ij-1)*6+1:6*ij,dofs_here) = dinamico_adj(eta_plus_here)*S((ij-1)*6+1:6*ij,dofs_here)+dS_dq_qd((ij-1)*6+1:6*ij,dofs_here);
            L((ij-1)*6+1:6*ij,dofs_here) = dinamico_adj(etad_plus_here)*S((ij-1)*6+1:6*ij,dofs_here)+dinamico_adj(eta_plus_here)*R((ij-1)*6+1:6*ij,dofs_here)...
                                          +dinamico_adj(eta_here)*dS_dq_qd((ij-1)*6+1:6*ij,dofs_here)+dSd_dq_qd((ij-1)*6+1:6*ij,dofs_here)+dS_dq_qdd((ij-1)*6+1:6*ij,dofs_here); % L is Q without gravity
            Q((ij-1)*6+1:6*ij,dofs_here) = L((ij-1)*6+1:6*ij,dofs_here)-dinamico_adj(dinamico_Adjoint(ginv(g_here))*G)*S((ij-1)*6+1:6*ij,dofs_here);
            Y((ij-1)*6+1:6*ij,dofs_here) = R((ij-1)*6+1:6*ij,dofs_here)+Sd((ij-1)*6+1:6*ij,:)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here);
            
            g_here  = g_here*gstep((ij-1)*4+1:4*ij,:);

            J_here    = Adgstepinv((ij-1)*6+1:6*ij,:)*(J_here+S((ij-1)*6+1:6*ij,:));
            eta_here  = Adgstepinv((ij-1)*6+1:6*ij,:)*(eta_plus_here);
            Jd_here   = Adgstepinv((ij-1)*6+1:6*ij,:)*(Jd_here+Sd((ij-1)*6+1:6*ij,:)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here));
            etad_here = Adgstepinv((ij-1)*6+1:6*ij,:)*(etad_plus_here);
        
            R_Bhere = Adgstepinv((ij-1)*6+1:6*ij,:)*(R_Bhere+R((ij-1)*6+1:6*ij,:));
            L_Bhere = Adgstepinv((ij-1)*6+1:6*ij,:)*(L_Bhere+L((ij-1)*6+1:6*ij,:));
            Q_Bhere = Adgstepinv((ij-1)*6+1:6*ij,:)*(Q_Bhere+Q((ij-1)*6+1:6*ij,:));

            g((i_sig-1)*4+1:4*i_sig,:) = g_here;

            J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
            eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
            Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
            etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;
        
            L_B((i_sig-1)*6+1:6*i_sig,:) = L_Bhere;
            R_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere;
            Q_B((i_sig-1)*6+1:6*i_sig,:) = Q_Bhere;
            Y_B((i_sig-1)*6+1:6*i_sig,:) = R_Bhere+Jd_here;

            ij = ij+1;
            i_sig = i_sig+1;

        end

        gf = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));

        g_here  = g_here*gf;

        J_here    = Ad_gf_inv*J_here;
        eta_here  = Ad_gf_inv*eta_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        etad_here = Ad_gf_inv*etad_here;
    
        R_Bhere = Ad_gf_inv*R_Bhere;
        L_Bhere = Ad_gf_inv*L_Bhere;
        Q_Bhere = Ad_gf_inv*Q_Bhere;

        if ~Linkage.OneBasis
            dof_start = dof_start+dof_here;
        end
    end

    g_tip((i-1)*4+1:i*4,:) = g_here;

    J_tip((i-1)*6+1:i*6,:)    = J_here;
    eta_tip((i-1)*6+1:i*6,:)  = eta_here;
    Jd_tip((i-1)*6+1:i*6,:)   = Jd_here;
    etad_tip((i-1)*6+1:i*6,:) = etad_here;

    R_Btip((i-1)*6+1:i*6,:) = R_Bhere;
    L_Btip((i-1)*6+1:i*6,:) = L_Bhere;
    Q_Btip((i-1)*6+1:i*6,:) = Q_Bhere;

end

%% Point Wrench 

for ip=1:Linkage.np
    Fp_here = Linkage.Fp_vec{ip}(0);
    i_sig = Linkage.Fp_sig(ip);
    I_theta = diag([1 1 1 0 0 0]);

    if ~Linkage.LocalWrench(i) %if in the global frame
        g_here = g((i_sig-1)*4+1:i_sig*4,:);
        g_here(1:3,4) = zeros(3,1); %only rotational part
        Fp_here = (dinamico_Adjoint(g_here))'*Fp_here; %rotated into the local frame. Adj' = coAdj^-1
        dFp_dq = dinamico_coadj(Fp_here)*I_theta*J((i_sig-1)*6+1:i_sig*6,:);
        dID_dq = dID_dq-J((i_sig-1)*6+1:i_sig*6,:)'*dFp_dq; %external force hence negative
    end

    Fk((i_sig-1)*6+1:i_sig*6) = Fk((i_sig-1)*6+1:i_sig*6)+Fp_here;
end

%% Custom External Force

if Linkage.CEF
    [Fext,dFext_dq]  = CustomExtForce(Linkage,q,g,J,t,qd,Jdot); %should be point wrench in local frame and its derivatives
    Fk = Fk+Fext;
    for i_sig=1:Linkage.nsig
        dID_dq = dID_dq-J((i_sig-1)*6+1:i_sig*6,:)'*dFext_dq((i_sig-1)*6+1:i_sig*6,:);
    end
end

%% Closed Chain Constraint

if Linkage.nCLj>0

    A = zeros(Linkage.CLprecompute.nCLp,ndof);
    if index==1
        Ad = zeros(Linkage.CLprecompute.nCLp,ndof);
        T_BS = Linkage.T_BS;
    end
    
    il_start = 1; %starting index of current closed chain joint lambda
    
    for iCLj=1:Linkage.nCLj %loop over all closed-chain joints
        
        i_sigA = Linkage.CLprecompute.i_sigA(iCLj); %significant point corresponding to A
        i_sigB = Linkage.CLprecompute.i_sigB(iCLj); %significant point corresponding to B
        Phi_p = Linkage.CLprecompute.Phi_p{iCLj}; % Basis of constraint force
        nl = size(Phi_p,2); %total number of constraint forces in the iCLj th closed chain joint
        
        if i_sigA>0
            if Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).linktype=='s'
                g_sigA2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).gf{end}*Linkage.gACLj{iCLj}; %Transformation from significant point to the closed loop joint
            else
                g_sigA2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLA(iCLj))).gf*Linkage.gACLj{iCLj};
            end
            gA = g((i_sigA-1)*4+1:i_sigA*4,:)*g_sigA2CLj;
            Ad_inv_gA2CLj = dinamico_Adjoint(ginv(g_sigA2CLj));
            JA = Ad_inv_gA2CLj*J((i_sigA-1)*6+1:i_sigA*6,:);
            if index==1
                eta_A  = Ad_inv_gA2CLj*eta((i_sigA-1)*6+1:i_sigA*6);
                etad_A = Ad_inv_gA2CLj*etad((i_sigA-1)*6+1:i_sigA*6);
    
                JdA       = Ad_inv_gA2CLj*Jd((i_sigA-1)*6+1:i_sigA*6,:);
                deta_dqA  = Ad_inv_gA2CLj*R_B((i_sigA-1)*6+1:i_sigA*6,:); %RB is deta_dq
                detad_dqA = Ad_inv_gA2CLj*(L_B((i_sigA-1)*6+1:i_sigA*6,:)-dinamico_adj(eta((i_sigA-1)*6+1:i_sigA*6))*R_B((i_sigA-1)*6+1:i_sigA*6,:)); %LB-ad(eta)*RB is detad_dq
                dpsi_dqdA = Ad_inv_gA2CLj(Y_B((i_sigA-1)*6+1:i_sigA*6,:)-dinamico_adj(eta((i_sigA-1)*6+1:i_sigA*6))*J((i_sigA-1)*6+1:i_sigA*6,:)); %YB-ad(eta)*J is detad_dqd = dpsi_dqd
            end
        else %if ground
            gA = Linkage.gACLj{iCLj};
            JA = zeros(6,ndof);
            if index==1
                eta_A  = zeros(6,1);
                etad_A = zeros(6,1);
    
                JdA       = zeros(6,ndof);
                deta_dqA  = zeros(6,ndof);
                detad_dqA = zeros(6,ndof);
                dpsi_dqdA = zeros(6,ndof);
            end
        end
    
        if i_sigB>0
            if Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).linktype=='s'
                g_sigB2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).gf{end}*Linkage.gBCLj{iCLj}; %Transformation from significant point to the closed loop joint
            else
                g_sigB2CLj = Linkage.VLinks(Linkage.LinkIndex(Linkage.iCLB(iCLj))).gf*Linkage.gBCLj{iCLj};
            end
            gB = g((i_sigB-1)*4+1:i_sigB*4,:)*g_sigB2CLj;
            Ad_inv_gB2CLj = dinamico_Adjoint(ginv(g_sigB2CLj));
            JB = Ad_inv_gB2CLj*J((i_sigB-1)*6+1:i_sigB*6,:);
            if index==1
                eta_B  = Ad_inv_gA2CLj*eta((i_sigB-1)*6+1:i_sigB*6,:);
    
                JdB       = Ad_inv_gB2CLj*Jd((i_sigB-1)*6+1:i_sigB*6,:);
                deta_dqB  = Ad_inv_gB2CLj*R_B((i_sigB-1)*6+1:i_sigB*6,:);
                detad_dqB = Ad_inv_gB2CLj*(L_B((i_sigB-1)*6+1:i_sigB*6,:)-dinamico_adj(eta((i_sigB-1)*6+1:i_sigB*6))*R_B((i_sigB-1)*6+1:i_sigB*6,:));
                dpsi_dqdB = Ad_inv_gB2CLj(Y_B((i_sigB-1)*6+1:i_sigB*6,:)-dinamico_adj(eta((i_sigB-1)*6+1:i_sigB*6))*J((i_sigB-1)*6+1:i_sigB*6,:));
            end
        else %if ground
            gB = Linkage.gBCLj{iCLj};
            JB = zeros(6,ndof);
            if index==1
                eta_B  = zeros(6,1);
    
                JdB       = zeros(6,ndof);
                deta_dqB  = zeros(6,ndof);
                detad_dqB = zeros(6,ndof);
                dpsi_dqdB = zeros(6,ndof);
            end
        end
        
        %constraint force is in the B frame. Hence we need to bring all to B frame
        gBA = ginv(gB)*gA;
        OmegaBA = piecewise_logmap(gBA); %twist vector from B to A in R^6
        Adj_gBA = dinamico_Adjoint(gBA);
    
        %Bringing quantities to B frame (note this step)
        JA = Adj_gBA*JA;
        diffJAB = JA-JB; %in B frame

        A(il_start:il_start+nl-1,:) = Phi_p'*diffJAB;

        [~,T_RodBA] = variable_expmap_gTg_mex(OmegaBA); %Tangent operator of the twist vector
        if index==1
            %Bringing quantities to B frame (note this step)
            eta_A = Adj_gBA*eta_A;
            etad_A = Adj_gBA*etad_A;
            deta_dqA = Adj_gBA*deta_dqA;
    
            etaBA = eta_A-eta_B; %relative velocity of A wrt B in B
            diffJdAB = Adj_gBA*JdA-JdB;
            diffdeta_dqAB  = deta_dqA-deta_dqB;
            diffdetad_dqAB = Adj_gBA*detad_dqA-detad_dqB;
            diffdpsi_dqdAB = Adj_gBA*dpsi_dqdA-dpsi_dqdB;
        
            Ad(il_start:il_start+nl-1,:) = Phi_p'*(diffJdAB+dinamico_adj(etaBA)*JA);
            % e = A*qdd+Ad*qd+2/T*A*qd+2/T^2*e = Phi_p'(Adj_gBA*etad_A - etad_B - adj_eta_B*Adj_gBA*eta_A + 2/T*(Adj_gBA*eta_A - eta_B) + 2/T^2*OmegaBA)
            de_dq(il_start:il_start+nl-1,:) = Phi_p'*(-dinamico_adj(etad_A)*diffJAB + diffdetad_dqAB + dinamico_adj(eta_A)*deta_dqB+dinamico_adj(eta_B)*dinamico_adj(eta_A)*diffJAB-dinamico_adj(eta_B)*deta_dqA...
                                             +2/T_BS*(-dinamico_adj(eta_A)*diffJAB+diffdeta_dqAB)...
                                             +1/T_BS^2*T_RodBA\diffJAB);
            de_dqd(il_start:il_start+nl-1,:) = Phi_p'*(diffdpsi_dqdAB+dinamico_adj(eta_A)*JB-dinamico_adj(eta_B)*JA+2/T_BS*diffJAB);
        else
            de_dq(il_start:il_start+nl-1,:) = Phi_p'*(T_RodBA\diffJAB);
        end
    
        F_lambda = Phi_p*lambda(il_start:il_start+nl-1); %Note that -F_lambda acts on B
    
        if i_sigA>0
            FlA = dinamico_coAdjoint(g_sigA2CLj)*Adj_gBA'*F_lambda; %+F_lambda at B is first transformed to A (Adj_gBA' is coAdj_gAB) then to the computational point
            Fk((i_sigA-1)*6+1:i_sigA*6) = Fk((i_sigA-1)*6+1:i_sigA*6)+FlA;
            dFlA_dq = -Adj_gBA'*dinamico_coadjbar(F_lambda)*diffJAB; %Adj_gBA'= coAdj_gBA^-1
            dID_dq = dID_dq-J((i_sigA-1)*6+1:i_sigA*6,:)'*dFlA_dq; %external force hence negative
        end
    
        if i_sigB>0
            FlB = -dinamico_coAdjoint(g_sigB2CLj)*F_lambda; %bringing from CLj to computational point
            Fk((i_sigB-1)*6+1:i_sigB*6) = Fk((i_sigB-1)*6+1:i_sigB*6)+FlB;
        end
        
        il_start = il_start+nl;
    end

    dID_dlambda = -A';
    if index==1
        de_dqdd = A;
    end

end

%A should be FULL ROW RANK Matrix. Otherwise Problem!

%% Backward Pass

%Start from the last link, computationl point, joint and dof
i_sig = nsig;
ij = nj;
dof_start = ndof;

%for branched chain computation
M_C_tip = zeros(N*6,6);
N_C_tip = zeros(N*6,6);
F_C_tip = zeros(N*6,1);
P_S_tip = zeros(N*6,ndof);
U_S_tip = zeros(N*6,ndof);
V_S_tip = zeros(N*6,ndof);
W_S_tip = zeros(N*6,ndof);

for i=N:-1:1 %backwards

    M_C = M_C_tip(6*(i-1)+1:6*i,:);
    N_C = N_C_tip(6*(i-1)+1:6*i,:);
    F_C = F_C_tip(6*(i-1)+1:6*i);
    P_S = P_S_tip(6*(i-1)+1:6*i,:);
    U_S = U_S_tip(6*(i-1)+1:6*i,:);
    V_S = V_S_tip(6*(i-1)+1:6*i,:);
    W_S = W_S_tip(6*(i-1)+1:6*i,:);
    
    G = Linkage.G;
    if ~Linkage.Gravity
        G = G*0;
    elseif Linkage.UnderWater %G is reduced by a factor of 1-rho_water/rho_body
        G=G*(1-Linkage.Rho_water/Linkage.VLinks(Linkage.LinkIndex(i)).Rho);
    end
    
    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
        %tip to CM
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf; 
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        coAd_gf   = Ad_gf_inv';
        
        M_C = coAd_gf*M_C*Ad_gf_inv;
        N_C = coAd_gf*N_C*Ad_gf_inv;
        F_C = coAd_gf*F_C;
        P_S = coAd_gf*P_S;
        U_S = coAd_gf*U_S;
        V_S = coAd_gf*V_S;
        W_S = coAd_gf*W_S;

        i_sig = i_sig-1; %at i_sig+1 is called

        %CM to base
        gi        = Linkage.VLinks(Linkage.LinkIndex(i)).gi; 
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        coAd_gi   = Ad_gi_inv';
        
        M_ap1 = Linkage.VLinks(Linkage.LinkIndex(i)).M; %at alpha + 1: ap1
        F_ap1 = -M_ap1*dinamico_Adjoint(ginv(g(i_sig*4+1:4*(i_sig+1),:)))*G... %Gravity (external hence -ve)
                -Fk(i_sig*6+1:6*(i_sig+1)); %External point forces

        if Linkage.UnderWater
            M_ap1 = M_ap1+Linkage.M_added(i_sig*6+1:(i_sig+1)*6,:); %Including Added Mass
            %Including Drag lift force
            DL_here = Linkage.DL(i_sig*6+1:(i_sig+1)*6,:); %drag lift matrix
            v_here_norm = norm(eta(i_sig*6+4:6*(i_sig+1)));
            if v_here_norm>0
                DL_here_star = DL_here*(1/v_here_norm*eta(i_sig*6+1:6*(i_sig+1))*eta(i_sig*6+1:6*(i_sig+1))'*I_v+v_here_norm*eye(6));
            else
                DL_here_star = zeros(6,6);
            end

            N_ap1 = dinamico_coadjbar(M_ap1*eta(i_sig*6+1:6*(i_sig+1)))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1-M_ap1*dinamico_adj(eta(i_sig*6+1:6*(i_sig+1)))+DL_here_star;
            F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1))...%inertial and Coriolis; 
                   +DL_here*v_here_norm*eta(i_sig*6+1:6*(i_sig+1));%Drag lift
        else
            N_ap1 = dinamico_coadjbar(M_ap1*eta(i_sig*6+1:6*(i_sig+1)))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1-M_ap1*dinamico_adj(eta(i_sig*6+1:6*(i_sig+1)));
            F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1));%inertial and Coriolis;
        end 

        M_C = coAd_gi*(M_ap1+M_C)*Ad_gi_inv;
        N_C = coAd_gi*(N_ap1+N_C)*Ad_gi_inv;
        F_C = coAd_gi*(F_ap1+F_C);
        P_S = coAd_gi*P_S;
        U_S = coAd_gi*U_S;
        V_S = coAd_gi*V_S;
        W_S = coAd_gi*W_S;

        %at X=1 of the rigid joint 
        M_ap1 = zeros(6,6); %at base
        N_ap1 = zeros(6,6); 
        F_ap1 = zeros(6,1);
    else
        for j=Linkage.VLinks(Linkage.LinkIndex(i)).npie-1:-1:1

            gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j}; 
            Ad_gf_inv = dinamico_Adjoint(ginv(gf));
            coAd_gf   = Ad_gf_inv';
            
            M_C = coAd_gf*M_C*Ad_gf_inv;
            N_C = coAd_gf*N_C*Ad_gf_inv;
            F_C = coAd_gf*F_C;
            P_S = coAd_gf*P_S;
            U_S = coAd_gf*U_S;
            V_S = coAd_gf*V_S;
            W_S = coAd_gf*W_S;

            dof_here = Linkage.CVRods{i}(j+1).dof;
            dofs_here = dof_start-dof_here+1:dof_start;
            Ws = Linkage.CVRods{i}(j+1).Ws;
            nip = Linkage.CVRods{i}(j+1).nip;
            i_sig = i_sig-1; %i_sig+1 is called

            for ii=nip-1:-1:1
            
                coAdgstep = Adgstepinv((ij-1)*6+1:6*ij,:)';
            
                M_ap1 = Ws(ii+1)*Linkage.CVRods{i}(2).Ms(ii*6+1:6*(ii+1),:); %at alpha + 1. multiplied with weight
                F_ap1 = -M_ap1*dinamico_Adjoint(ginv(g(i_sig*4+1:4*(i_sig+1),:)))*G... %Gravity (external hence -ve)
                        -Fk(i_sig*6+1:6*(i_sig+1)); %External point forces

                if Linkage.UnderWater
                    M_ap1 = M_ap1+Ws(ii+1)*Linkage.M_added(i_sig*6+1:(i_sig+1)*6,:); %Including Added Mass
                    %Including Drag lift force
                    DL_here = Ws(ii+1)*Linkage.DL(i_sig*6+1:(i_sig+1)*6,:); %drag lift matrix
                    v_here_norm = norm(eta(i_sig*6+4:6*(i_sig+1)));
                    if v_here_norm>0
                        DL_here_star = DL_here*(1/v_here_norm*eta(i_sig*6+1:6*(i_sig+1))*eta(i_sig*6+1:6*(i_sig+1))'*I_v+v_here_norm*eye(6));
                    else
                        DL_here_star = zeros(6,6);
                    end
    
                    N_ap1 = dinamico_coadjbar(M_ap1*eta(i_sig*6+1:6*(i_sig+1)))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1-M_ap1*dinamico_adj(eta(i_sig*6+1:6*(i_sig+1)))+DL_here_star;
                    F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1))...%inertial and Coriolis; 
                           +DL_here*v_here_norm*eta(i_sig*6+1:6*(i_sig+1));%Drag lift
                else
                    N_ap1 = dinamico_coadjbar(M_ap1*eta(i_sig*6+1:6*(i_sig+1)))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1-M_ap1*dinamico_adj(eta(i_sig*6+1:6*(i_sig+1)));
                    F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1));%inertial and Coriolis;
                end 
    
                M_C = coAdgstep*(M_ap1+M_C)*Adgstepinv((ij-1)*6+1:6*ij,:);
                N_C = coAdgstep*(N_ap1+N_C)*Adgstepinv((ij-1)*6+1:6*ij,:);
                F_C = coAdgstep*(F_ap1+F_C);
    
                %split to two steps to optimize computation
                P_S = coAdgstep*P_S;
                U_S = coAdgstep*U_S;
                V_S = coAdgstep*V_S;
                W_S = coAdgstep*W_S;

                P_S(:,dofs_here) = P_S(:,dofs_here)+dinamico_coadjbar(F_C)*S((ij-1)*6+1:6*ij,dofs_here);
                U_S(:,dofs_here) = U_S(:,dofs_here)+N_C*R((ij-1)*6+1:6*ij,dofs_here)+M_C*Q((ij-1)*6+1:6*ij,dofs_here);
                V_S(:,dofs_here) = V_S(:,dofs_here)+N_C*S((ij-1)*6+1:6*ij,dofs_here)+M_C*Y((ij-1)*6+1:6*ij,dofs_here);
                W_S(:,dofs_here) = W_S(:,dofs_here)+M_C*S((ij-1)*6+1:6*ij,dofs_here);

                if Linkage.Z_order==4
                    Phi_Z1 = Linkage.CVRods{i}(j+1).Phi_Z1((ii-1)*6+1:6*ii,:);
                    Phi_Z2 = Linkage.CVRods{i}(j+1).Phi_Z2((ii-1)*6+1:6*ii,:);
        
                    dSTdq_FC = compute_dSTdqFC_Z4_mex(h(ij),Omega(:,ij),Phi_Z1,Phi_Z2,Z((ij-1)*6+1:6*ij,dofs_here),T((ij-1)*6+1:6*ij,:),f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),F_C);
                else
                    dSTdq_FC = compute_dSTdqFC_Z2R_mex(h(ij),Omega(:,ij),Z((ij-1)*6+1:6*ij,dofs_here),T((ij-1)*6+1:6*ij,:),f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),F_C);
                end

                dSTdq_FC_full = zeros(dof_here,ndof);
                dSTdq_FC_full(:,dofs_here) = dSTdq_FC;
                
                dID_dq(dofs_here,:) = dID_dq(dofs_here,:)+dSTdq_FC_full+S((ij-1)*6+1:6*ij,dofs_here)'*(N_C*R_B((isig-1)*6+1:6*isig,:)+M_C*Q_B((isig-1)*6+1:6*isig,:)+U_S+P_S);
                dID_dqd(dofs_here,:) = dID_dqd(dofs_here,:)+S((ij-1)*6+1:6*ij,dofs_here)'*(N_C*J((isig-1)*6+1:6*isig,:)+M_C*Y_B((isig-1)*6+1:6*isig,:)+V_S);
                dID_dqdd(dofs_here,:) = dID_dqdd(dofs_here,:)+S((ij-1)*6+1:6*ij,dofs_here)'*(M_C*J((isig-1)*6+1:6*isig,:)+W_S);
    
                i_sig = i_sig-1;
                ij = ij-1;
            
            end
            
            M_ap1 = Ws(1)*Linkage.CVRods{i}(2).Ms(1:6,:); %at X = 0 usually Ws is 0
            F_ap1 = -M_ap1*dinamico_Adjoint(ginv(g(i_sig*4+1:4*(i_sig+1),:)))*G... %Gravity (external hence -ve)
                    -Fk(i_sig*6+1:6*(i_sig+1)); %External point forces
            
            if Linkage.UnderWater
                M_ap1 = M_ap1+Ws(1)*Linkage.M_added(i_sig*6+1:(i_sig+1)*6,:); %Including Added Mass
                %Including Drag lift force
                DL_here = Ws(1)*Linkage.DL(i_sig*6+1:(i_sig+1)*6,:); %drag lift matrix
                v_here_norm = norm(eta(i_sig*6+4:6*(i_sig+1)));
                if v_here_norm>0
                    DL_here_star = DL_here*(1/v_here_norm*eta(i_sig*6+1:6*(i_sig+1))*eta(i_sig*6+1:6*(i_sig+1))'*I_v+v_here_norm*eye(6));
                else
                    DL_here_star = zeros(6,6);
                end

                N_ap1 = dinamico_coadjbar(M_ap1*eta(i_sig*6+1:6*(i_sig+1)))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1-M_ap1*dinamico_adj(eta(i_sig*6+1:6*(i_sig+1)))+DL_here_star;
                F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1))...%inertial and Coriolis; 
                       +DL_here*v_here_norm*eta(i_sig*6+1:6*(i_sig+1));%Drag lift
            else
                N_ap1 = dinamico_coadjbar(M_ap1*eta(i_sig*6+1:6*(i_sig+1)))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1-M_ap1*dinamico_adj(eta(i_sig*6+1:6*(i_sig+1)));
                F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1));%inertial and Coriolis;
            end
           
            gi        = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j}; %fixed transforamtion from X=0 of the rod to X=1 of joint
            Ad_gi_inv = dinamico_Adjoint(ginv(gi));
            coAd_gi   = Ad_gi_inv';
            
            M_C = coAd_gi*(M_ap1+M_C)*Ad_gi_inv;
            N_C = coAd_gi*(N_ap1+N_C)*Ad_gi_inv;
            F_C = coAd_gi*(F_ap1+F_C);
            P_S = coAd_gi*P_S;
            U_S = coAd_gi*U_S;
            V_S = coAd_gi*V_S;
            W_S = coAd_gi*W_S;

            if ~Linkage.OneBasis
                dof_start=dof_start-dof_here;
            end
        end
        %at X=1 of the rigid joint 
        M_ap1 = zeros(6,6);
        N_ap1 = zeros(6,6); 
        F_ap1 = zeros(6,1);
    end

    %joint
    dof_here = Linkage.CVRods{i}(1).dof;

    if dof_here>0
        dofs_here = dof_start-dof_here+1:dof_start;
        coAdgstep = Adgstepinv((ij-1)*6+1:6*ij,:)';
        
        M_C = coAdgstep*(M_ap1+M_C)*Adgstepinv((ij-1)*6+1:6*ij,:);
        N_C = coAdgstep*(N_ap1+N_C)*Adgstepinv((ij-1)*6+1:6*ij,:);
        F_C = coAdgstep*(F_ap1+F_C);

        %split to two steps to optimize computation
        P_S = coAdgstep*P_S;
        U_S = coAdgstep*U_S;
        V_S = coAdgstep*V_S;
        W_S = coAdgstep*W_S;

        P_S(:,dofs_here) = P_S(:,dofs_here)+dinamico_coadjbar(F_C)*S((ij-1)*6+1:6*ij,dofs_here);
        U_S(:,dofs_here) = U_S(:,dofs_here)+N_C*R((ij-1)*6+1:6*ij,dofs_here)+M_C*Q((ij-1)*6+1:6*ij,dofs_here);
        V_S(:,dofs_here) = V_S(:,dofs_here)+N_C*S((ij-1)*6+1:6*ij,dofs_here)+M_C*Y((ij-1)*6+1:6*ij,dofs_here);
        W_S(:,dofs_here) = W_S(:,dofs_here)+M_C*S((ij-1)*6+1:6*ij,dofs_here);

        dSTdq_FC = compute_dSTdqFC_Z2R_mex(dof_here,Omega(:,ij),Z((ij-1)*6+1:6*ij,dofs_here),f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),F_C);
        dSTdq_FC_full = zeros(dof_here,ndof);
        dSTdq_FC_full(:,dofs_here) = dSTdq_FC;
    
        dID_dq(dofs_here,:) = dID_dq(dofs_here,:)+dSTdq_FC_full+S((ij-1)*6+1:6*ij,dofs_here)'*(N_C*R_B((isig-1)*6+1:6*isig,:)+M_C*Q_B((isig-1)*6+1:6*isig,:)+U_S+P_S);
        dID_dqd(dofs_here,:) = dID_dqd(dofs_here,:)+S((ij-1)*6+1:6*ij,dofs_here)'*(N_C*J((isig-1)*6+1:6*isig,:)+M_C*Y_B((isig-1)*6+1:6*isig,:)+V_S);
        dID_dqdd(dofs_here,:) = dID_dqdd(dofs_here,:)+S((ij-1)*6+1:6*ij,dofs_here)'*(M_C*J((isig-1)*6+1:6*isig,:)+W_S);
    end
    
    i_sig = i_sig-1;
    ij = ij-1;
    if ~Linkage.OneBasis
        dof_start=dof_start-dof_here;
    end

    % projecting to the tip of parent link
    ip = iLpre(i); %index of previous link
    if ip>0
        Ad_g_ini_inv = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)));
        coAd_g_ini = Ad_g_ini_inv';
    
        M_C_tip(6*(ip-1)+1:6*ip,:) = M_C_tip(6*(ip-1)+1:6*ip,:)+coAd_g_ini*M_C*Ad_g_ini_inv;
        N_C_tip(6*(ip-1)+1:6*ip,:) = N_C_tip(6*(ip-1)+1:6*ip,:)+coAd_g_ini*N_C*Ad_g_ini_inv;
        F_C_tip(6*(ip-1)+1:6*ip)   = F_C_tip(6*(ip-1)+1:6*ip)+coAd_g_ini*F_C;
        P_S_tip(6*(ip-1)+1:6*ip,:) = P_S_tip(6*(ip-1)+1:6*ip,:)+coAd_g_ini*P_S;
        U_S_tip(6*(ip-1)+1:6*ip,:) = U_S_tip(6*(ip-1)+1:6*ip,:)+coAd_g_ini*U_S;
        V_S_tip(6*(ip-1)+1:6*ip,:) = V_S_tip(6*(ip-1)+1:6*ip,:)+coAd_g_ini*V_S;
        W_S_tip(6*(ip-1)+1:6*ip,:) = W_S_tip(6*(ip-1)+1:6*ip,:)+coAd_g_ini*W_S;

    end

end

%% Actuation

if Linkage.Damped
    D = Linkage.D;
else
    D = 0;
end
K = Linkage.K; 

dtau_dq = -K; %more to add
dtau_dqd = -D;

if Linkage.Actuated

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
            ij = Linkage.ActuationPrecompute.ij_act(ii);
            S_here = S((ij-1)*6+1:ij*6,i_jactq(i_u:i_u+dof_here-1));
            coAdgstep_here = Adgstepinv((ij-1)*6+1:6*ij,:)';
            Phi_here = Linkage.CVRods{i}(1).Phi;
            coAdgstepPhi_here = coAdgstep_here*Phi_here;
            B(dofs_here,us_here) = S_here'*coAdgstepPhi_here;
            Fk0 = coAdgstepPhi_here*u(us_here);
            dBu_dq_here = compute_dBu_dq_joint(Omega(:,ij),S_here,Phi_here,f(:,ij),fd(:,ij),adjOmegap((ij-1)*24+1:24*ij,:),Fk0); %partial derivative of B*u for the joint wrt q_here. check also convert to C to make faster
            dtau_dq(dofs_here,dofs_here) = dtau_dq(dofs_here,dofs_here)+dBu_dq_here;
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
                    dBu_dq_here = zeros(dof_here,dof_here);
                    for ii=1:nip
                        if Ws(ii)>0
                            Phi = Linkage.CVRods{i}(j+1).Phi((ii-1)*6+1:ii*6,:);
                            xi  = Phi*q(dofs_here)+Linkage.CVRods{i}(j+1).xi_star((ii-1)*6+1:ii*6,1);
                            Phi_a = zeros(6,na_here);
                            PHI_AU = zeros(6,6);
                            xihat_123  = [0 -xi(3) xi(2) xi(4);xi(3) 0 -xi(1) xi(5);-xi(2) xi(1) 0 xi(6)];%4th row is avoided to speedup calculation
                            for ia =Linkage.i_sact{i}{j}
                                dc = Linkage.dc{ia,i}{j}(:,ii);
                                dcp = Linkage.dcp{ia,i}{j}(:,ii);
                                [Phi_a(:,ia),PHI_AU_ia] = SoftActuator(u(n_jact+ia),dc,dcp,xihat_123);
                                PHI_AU = PHI_AU+PHI_AU_ia;
                            end
                            B_here = B_here+Ws(ii)*Phi'*Phi_a;
                            dBu_dq_here = dBu_dq_here+Ws(ii)*Phi'*PHI_AU*Phi;
                        end
                    end
                    B(dofs_here,n_jact+Linkage.i_sact{i}{j}) = B_here;
                    dtau_dq(dofs_here,dofs_here) = dtau_dq(dofs_here,dofs_here)+dBu_dq_here;
                end
            end
        end
    end

    dtau_du = B;
end
%% Custom Soft Actuation

if Linkage.CA
    [Fact,dFact_dq] = CustomActuation(Linkage,q,g,J,0,zeros(ndof,1),zeros(6*nsig,1),zeros(6*nsig,ndof));
    [~,dtauC_dq] = CustomActuation_Qspace(Linkage,Fact,dFact_dq);
    dtau_dq = dtau_dq+dtauC_dq;
end

end