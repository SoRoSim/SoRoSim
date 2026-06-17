function [ID,tau,g,J] = RNEA(Linkage,t,q,qd,qdd,u)

persistent ID_last tau_last g_last J_last


% -------------------- cache getter --------------------
if ischar(Linkage) || (isstring(Linkage) && isscalar(Linkage))
    if strcmpi(string(Linkage),"getlast")
        ID = ID_last;
        tau = tau_last;

        g  = g_last;
        J  = J_last;
        return
    else
        error("dRNEA unknown mode. Use 'getlast'.");
    end
end

%For index 1 DAE e=e(q,qd,qdd). For index 3 DAE e=e(q)

N    = Linkage.N; %Total number of Links (including repetition)
ndof = Linkage.ndof; %Total dof of the Linkage
nsig = Linkage.nsig; %Total number of computational points
nj   = Linkage.nj; %Total number of joints (rigid and virtual soft joints (nip-1))
nact = Linkage.nact;

ID       = zeros(ndof,1); %Generalized Inertial+Coriolis-External 

%tau, dtau_dq, dtau_dqd, dtau_du (B) are initialized later

Fk = zeros(6*nsig,1); %External point force at every significant point

%% Forward kinematic pass: evaluate kinematic, and derivatives terms at every virtual joint and significant points

gstep = zeros(4*nj,4);      %Linkageansformation from X_alpha to X_alpha+1 (0 to 1 for rigid joints)
Adgstepinv = zeros(6*nj,6); %As the name says!

S = zeros(6*nj,ndof);         %S is joint motion subspace (sparse matrix for a reason)
Sd = zeros(6*nj,ndof);        %Sd is time derivative of joint motion subspace (sparse matrix for a reason)

g = zeros(4*nsig,4);    %Fwd kinematics

J    = zeros(6*nsig,ndof); %Jacobian (J is S_B)
Jd   = zeros(6*nsig,ndof); %Jacobian dot (Jd is Sd_B)
eta  = zeros(6*nsig,1);    %velocity twist
etad = zeros(6*nsig,1);    %acceleration twist

%For branched chain computation
g_tip = repmat(eye(4),N,1);

J_tip    = zeros(N*6,ndof);
eta_tip  = zeros(N*6,1);
Jd_tip   = zeros(N*6,ndof);
etad_tip = zeros(N*6,1);


dof_start = 1; %starting dof of current rod
i_sig = 1;     %current computational point index
ij = 1;        %current virtual joint index

iLpre = Linkage.iLpre;
g_ini = Linkage.g_ini;

for i=1:N

    if iLpre(i)>0
        g_here = g_tip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        Ad_g_ini_inv = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)));

        J_here    = Ad_g_ini_inv*J_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        Jd_here   = Ad_g_ini_inv*Jd_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        eta_here  = Ad_g_ini_inv*eta_tip((iLpre(i)-1)*6+1:iLpre(i)*6);
        etad_here = Ad_g_ini_inv*etad_tip((iLpre(i)-1)*6+1:iLpre(i)*6);
    else
        g_here = g_ini((i-1)*4+1:i*4,:);

        J_here    = zeros(6,ndof);
        Jd_here   = zeros(6,ndof);
        eta_here  = zeros(6,1);
        etad_here = zeros(6,1);
    end

    %Joint

    %0 of joint
    g((i_sig-1)*4+1:4*i_sig,:) = g_here;

    J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
    Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
    eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
    etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;

    dof_here  = Linkage.CVRods{i}(1).dof;
    dofs_here = dof_start:dof_start+dof_here-1;

    Phi_here = Linkage.CVRods{i}(1).Phi;
    xi_star  = Linkage.CVRods{i}(1).xi_star;

    %computation of kinematic and differential kinematic quantities at the rigid joint


    if dof_here==0 %fixed joint (N)
        gstep   = eye(4);
    else
        xi          = Phi_here*q(dofs_here)+xi_star;
        xid         = Phi_here*qd(dofs_here);

        [gstep((ij-1)*4+1:4*ij,:),Tg,Tgd] = variable_expmap_gTgTgd_mex(xi,xid);

        S((ij-1)*6+1:6*ij,dofs_here)  = Tg*Phi_here;
        Sd((ij-1)*6+1:6*ij,dofs_here) = Tgd*Phi_here;
    end

    Adgstepinv((ij-1)*6+1:6*ij,:) = dinamico_Adjoint(ginv(gstep((ij-1)*4+1:4*ij,:)));

    eta_plus_here  = eta_here+S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here); %eta of X=1 expressed in the frame of X=0
    etad_plus_here = etad_here+S((ij-1)*6+1:6*ij,dofs_here)*qdd(dofs_here)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here)+Sd((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here);
    
    % from 0 to 1 of rigid joint
    g_here  = g_here*gstep((ij-1)*4+1:4*ij,:);

    J_here    = Adgstepinv((ij-1)*6+1:6*ij,:)*(J_here+S((ij-1)*6+1:6*ij,:));
    Jd_here   = Adgstepinv((ij-1)*6+1:6*ij,:)*(Jd_here+Sd((ij-1)*6+1:6*ij,:)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,:));
    eta_here  = Adgstepinv((ij-1)*6+1:6*ij,:)*(eta_plus_here);
    etad_here = Adgstepinv((ij-1)*6+1:6*ij,:)*(etad_plus_here);
    
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
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        etad_here = Ad_gi_inv*etad_here;
    
        %CM is a significant point
        g((i_sig-1)*4+1:4*i_sig,:) = g_here;

        J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
        Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
        eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
        etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;

        i_sig = i_sig+1;

        %CM to tip
        gf = Linkage.VLinks(Linkage.LinkIndex(i)).gf; 
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));

        g_here  = g_here*gf;

        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        etad_here = Ad_gf_inv*etad_here;
 
    end

    for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1 %will run only if npie>1, ie for soft links

        dof_here  = Linkage.CVRods{i}(j+1).dof;
        dofs_here = dof_start:dof_start+dof_here-1;
        
        xi_star = Linkage.CVRods{i}(j+1).xi_star;
        nip     = Linkage.CVRods{i}(j+1).nip;
        Xs      = Linkage.CVRods{i}(j+1).Xs;
        ld      = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        
        gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j}; %X=1 of joint to first quadrature point at X=0 of the rod
        Ad_gi_inv = dinamico_Adjoint(ginv(gi)); %transformation from X=1 of joint or X=Ld of previous division to the X=0 of next division
        
        g_here  = g_here*gi;

        J_here    = Ad_gi_inv*J_here;
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        etad_here = Ad_gi_inv*etad_here;
    

        g((i_sig-1)*4+1:4*i_sig,:) = g_here;

        J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
        Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
        eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
        etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;

        i_sig  = i_sig+1;

        for ii=1:nip-1 
            

            H = (Xs(ii+1)-Xs(ii))*ld;
            
            if Linkage.Z_order==4
                
                fZ4 = ((sqrt(3)*H^2)/12); %just a factor

                xi_Z1here = xi_star(6*(ii-1)+1:6*ii,2); 
                xi_Z2here = xi_star(6*(ii-1)+1:6*ii,3);   
                Phi_Z1here = Linkage.CVRods{i}(j+1).Phi_Z1(6*(ii-1)+1:6*ii,:);
                Phi_Z2here = Linkage.CVRods{i}(j+1).Phi_Z2(6*(ii-1)+1:6*ii,:);

                if dof_here>0

                    xi_Z1here = Phi_Z1here*q(dofs_here)+xi_Z1here;
                    xi_Z2here = Phi_Z2here*q(dofs_here)+xi_Z2here;

                    xid_Z1here  = Phi_Z1here*qd(dofs_here);
                    ad_xi_Z1here = dinamico_adj(xi_Z1here);

                    Z_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                              fZ4*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);

                    Zd_here = 2*fZ4*dinamico_adj(xid_Z1here)*Phi_Z2here; %not technically correct, but this term is always multiplied with qd
                    Omegad_here   = Z_here*qd(dofs_here);

                else

                    ad_xi_Z1here = dinamico_adj(xi_Z1here);
                    Z_here  = [];
                    Zd_here = []; 
                    Omegad_here = zeros(6,1); 

                end

                Omega_here = (H/2)*(xi_Z1here+xi_Z2here)+...
                             fZ4*ad_xi_Z1here*xi_Z2here;
                      
            else % order 2
                
                xi_Zhere = xi_star(6*(ii-1)+1:6*ii,4);
                Phi_Zhere = Linkage.CVRods{i}(j+1).Phi_Z(6*(ii-1)+1:6*ii,:);

                if dof_here>0
                    xi_Zhere = Phi_Zhere*q(dofs_here)+xi_Zhere;
                    Z_here = H*Phi_Zhere;
                    Omegad_here = Z_here*qd(dofs_here);
                else
                    Z_here = H*Phi_Zhere;
                    Omegad_here   = zeros(6,1); 
                end
                Omega_here  = H*xi_Zhere;

            end

            [gstep((ij-1)*4+1:4*ij,:),Tg,Tgd] = variable_expmap_gTgTgd_mex(Omega_here,Omegad_here); % mex code, C program
            Adgstepinv((ij-1)*6+1:6*ij,:) = dinamico_Adjoint(ginv(gstep((ij-1)*4+1:4*ij,:)));

            if dof_here>0
                S((ij-1)*6+1:6*ij,dofs_here)  = Tg*Z_here;
                Sd((ij-1)*6+1:6*ij,dofs_here) = Tgd*Z_here;
                if Linkage.Z_order==4
                    Sd((ij-1)*6+1:6*ij,dofs_here) = Sd((ij-1)*6+1:6*ij,dofs_here)+Tg*Zd_here;
                end
            end
           
            eta_plus_here  = eta_here+S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here); %eta of next computational point expressed in the frame of current
            etad_plus_here = etad_here+S((ij-1)*6+1:6*ij,dofs_here)*qdd(dofs_here)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here)+Sd((ij-1)*6+1:6*ij,dofs_here)*qd(dofs_here);
            
            g_here  = g_here*gstep((ij-1)*4+1:4*ij,:);

            J_here    = Adgstepinv((ij-1)*6+1:6*ij,:)*(J_here+S((ij-1)*6+1:6*ij,:));
            Jd_here   = Adgstepinv((ij-1)*6+1:6*ij,:)*(Jd_here+Sd((ij-1)*6+1:6*ij,:)+dinamico_adj(eta_here)*S((ij-1)*6+1:6*ij,:));
            eta_here  = Adgstepinv((ij-1)*6+1:6*ij,:)*(eta_plus_here);
            etad_here = Adgstepinv((ij-1)*6+1:6*ij,:)*(etad_plus_here);

            g((i_sig-1)*4+1:4*i_sig,:) = g_here;

            J((i_sig-1)*6+1:6*i_sig,:)    = J_here;
            Jd((i_sig-1)*6+1:6*i_sig,:)   = Jd_here;
            eta((i_sig-1)*6+1:6*i_sig,:)  = eta_here;
            etad((i_sig-1)*6+1:6*i_sig,:) = etad_here;
      

            ij = ij+1;
            i_sig = i_sig+1;

        end

        gf = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));

        g_here  = g_here*gf;

        J_here    = Ad_gf_inv*J_here;
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        etad_here = Ad_gf_inv*etad_here;

        if ~Linkage.OneBasis
            dof_start = dof_start+dof_here;
        end
    end

    g_tip((i-1)*4+1:i*4,:) = g_here;

    J_tip((i-1)*6+1:i*6,:)    = J_here;
    Jd_tip((i-1)*6+1:i*6,:)   = Jd_here;
    eta_tip((i-1)*6+1:i*6,:)  = eta_here;
    etad_tip((i-1)*6+1:i*6,:) = etad_here;

end

%% Point Wrench 

for ip=1:Linkage.np
    Fp_here = Linkage.Fp_vec{ip}(t);
    i_sig = Linkage.Fp_sig(ip);

    if ~Linkage.LocalWrench(ip) %if in the global frame
        g_here = g((i_sig-1)*4+1:i_sig*4,:);
        g_here(1:3,4) = zeros(3,1); %only rotational part
        Fp_here = (dinamico_Adjoint(g_here))'*Fp_here; %rotated into the local frame. Adj' = coAdj^-1
    end

    Fk((i_sig-1)*6+1:i_sig*6) = Fk((i_sig-1)*6+1:i_sig*6)+Fp_here;
end

%% Custom External Force

if Linkage.CEF
    Fext  = CustomExtForce(Linkage,q,g,J,t,qd,Jdot); %should be point wrench in local frame and its derivatives
    Fk = Fk+Fext;
end

%% Backward Pass

%Start from the last link, computationl point, joint and dof
i_sig = nsig;
ij = nj;
dof_start = ndof;

%for branched chain computation
M_C_tip = zeros(N*6,6);
F_C_tip = zeros(N*6,1);

for i=N:-1:1 %backwards

    M_C = M_C_tip(6*(i-1)+1:6*i,:);
    F_C = F_C_tip(6*(i-1)+1:6*i);
    
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
        F_C = coAd_gf*F_C;

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

            F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1))...%inertial and Coriolis; 
                   +DL_here*v_here_norm*eta(i_sig*6+1:6*(i_sig+1));%Drag lift
        else
            F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1));%inertial and Coriolis;
        end 

        M_C = coAd_gi*(M_ap1+M_C)*Ad_gi_inv;
        F_C = coAd_gi*(F_ap1+F_C);

        %at X=1 of the rigid joint 
        M_ap1 = zeros(6,6); %at base
        F_ap1 = zeros(6,1);
    else
        for j=Linkage.VLinks(Linkage.LinkIndex(i)).npie-1:-1:1

            gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j}; 
            Ad_gf_inv = dinamico_Adjoint(ginv(gf));
            coAd_gf   = Ad_gf_inv';
            
            M_C = coAd_gf*M_C*Ad_gf_inv;
            F_C = coAd_gf*F_C;

            dof_here = Linkage.CVRods{i}(j+1).dof;
            dofs_here = dof_start-dof_here+1:dof_start;
            Ws = Linkage.CVRods{i}(j+1).Ws;
            nip = Linkage.CVRods{i}(j+1).nip;
            i_sig = i_sig-1; %i_sig+1 is called

            for ii=nip-1:-1:1
            
                coAdgstep = Adgstepinv((ij-1)*6+1:6*ij,:)';
            
                M_ap1 = Ws(ii+1)*Linkage.CVRods{i}(j+1).Ms(ii*6+1:6*(ii+1),:); %at alpha + 1. multiplied with weight
                F_ap1 = -M_ap1*dinamico_Adjoint(ginv(g(i_sig*4+1:4*(i_sig+1),:)))*G... %Gravity (external hence -ve)
                        -Fk(i_sig*6+1:6*(i_sig+1)); %External point forces

                if Linkage.UnderWater
                    M_ap1 = M_ap1+Ws(ii+1)*Linkage.M_added(i_sig*6+1:(i_sig+1)*6,:); %Including Added Mass
                    %Including Drag lift force
                    DL_here = Ws(ii+1)*Linkage.DL(i_sig*6+1:(i_sig+1)*6,:); %drag lift matrix
                    v_here_norm = norm(eta(i_sig*6+4:6*(i_sig+1)));
                    
                    F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1))...%inertial and Coriolis; 
                           +DL_here*v_here_norm*eta(i_sig*6+1:6*(i_sig+1));%Drag lift
                else
                    F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1));%inertial and Coriolis;
                end 
    
                M_C = coAdgstep*(M_ap1+M_C)*Adgstepinv((ij-1)*6+1:6*ij,:);
                F_C = coAdgstep*(F_ap1+F_C);

                
                ID(dofs_here,:) = ID(dofs_here,:)+S((ij-1)*6+1:6*ij,dofs_here)'*F_C;
                
                i_sig = i_sig-1;
                ij = ij-1;
            
            end
            
            M_ap1 = Ws(1)*Linkage.CVRods{i}(j+1).Ms(1:6,:); %at X = 0 usually Ws is 0
            F_ap1 = -M_ap1*dinamico_Adjoint(ginv(g(i_sig*4+1:4*(i_sig+1),:)))*G... %Gravity (external hence -ve)
                    -Fk(i_sig*6+1:6*(i_sig+1)); %External point forces
            
            if Linkage.UnderWater
                M_ap1 = M_ap1+Ws(1)*Linkage.M_added(i_sig*6+1:(i_sig+1)*6,:); %Including Added Mass
                %Including Drag lift force
                DL_here = Ws(1)*Linkage.DL(i_sig*6+1:(i_sig+1)*6,:); %drag lift matrix
                v_here_norm = norm(eta(i_sig*6+4:6*(i_sig+1)));
                
                F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1))...%inertial and Coriolis; 
                       +DL_here*v_here_norm*eta(i_sig*6+1:6*(i_sig+1));%Drag lift
            else
                F_ap1 = F_ap1+M_ap1*etad(i_sig*6+1:6*(i_sig+1))+dinamico_coadj(eta(i_sig*6+1:6*(i_sig+1)))*M_ap1*eta(i_sig*6+1:6*(i_sig+1));%inertial and Coriolis;
            end
           
            gi        = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j}; %fixed transforamtion from X=0 of the rod to X=1 of joint
            Ad_gi_inv = dinamico_Adjoint(ginv(gi));
            coAd_gi   = Ad_gi_inv';
            
            M_C = coAd_gi*(M_ap1+M_C)*Ad_gi_inv;
            F_C = coAd_gi*(F_ap1+F_C);

            if ~Linkage.OneBasis
                dof_start=dof_start-dof_here;
            end
        end
        %at X=1 of the rigid joint 
        M_ap1 = zeros(6,6);
        F_ap1 = zeros(6,1);
    end

    %joint
    dof_here = Linkage.CVRods{i}(1).dof;

    if dof_here>0
        dofs_here = dof_start-dof_here+1:dof_start;
        coAdgstep = Adgstepinv((ij-1)*6+1:6*ij,:)';
        
        M_C = coAdgstep*(M_ap1+M_C)*Adgstepinv((ij-1)*6+1:6*ij,:);
        F_C = coAdgstep*(F_ap1+F_C);

        ID(dofs_here,:) = ID(dofs_here,:)+S((ij-1)*6+1:6*ij,dofs_here)'*F_C;
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
        F_C_tip(6*(ip-1)+1:6*ip)   = F_C_tip(6*(ip-1)+1:6*ip)+coAd_g_ini*F_C;

    end

end

%% Actuation

if Linkage.Damped
    D = Linkage.D;
else
    D = 0;
end
K = Linkage.K; 
tau = -K*q-D*qd; %tau = Bu-Kq



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
                                Phi_a(:,ia_here) = SoftActuator(u(n_jact+ia),dc,dcp,xihat_123);
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

    tau = tau+B*u;

end

% -------------------- update cache for retrieval --------------------
ID_last    = ID;
tau_last    = tau;

g_last    = g;
J_last    = J;

end