%Single pass algorithm using D'Alembert-Kane method
%Last modified by Anup Teejo Mathew 20.01.2025

function Res=EquilibriumResidue(Linkage,x,action,staticsOptions) 
% x is a vector of unknowns. It has unknown q, unknown u and unknown lambdas. x = [q_u;u_u;lambda]
% input is a vector of known inputs. It has input values of u and q_joint. input = [u_k;q_k]

N    = Linkage.N;
ndof = Linkage.ndof;
nsig = Linkage.nsig; %Total number of computational points

q = x(1:ndof);
lambda = x(ndof+1:end);

if Linkage.CAI
    action = CustomActuatorInput(Linkage,x); %x is qul here
end

u = action;

if Linkage.Actuated
    nact = Linkage.nact;
    n_k  = Linkage.ActuationPrecompute.n_k; %number of q_joint inputs
    %if n_k>0 rearrangemetns are required to compute q and u in the correct format
    if n_k>0
        q(Linkage.ActuationPrecompute.index_q_u) = x(1:ndof-n_k);
        q(Linkage.ActuationPrecompute.index_q_k) = action(end-n_k+1:end);
        u(Linkage.ActuationPrecompute.index_u_k) = action(1:nact-n_k);
        u(Linkage.ActuationPrecompute.index_u_u) = x(ndof-n_k+1:ndof);
    end
end

F = zeros(ndof,1); 

%% Single Pass Algorithm using D'Alembert-Kane Projection

J = zeros(6*nsig,ndof);   %Jacobian (J is S_B)
g = zeros(4*nsig,4);      %Fwd kinematics

%For branched chain computation
g_tip  = repmat(eye(4),N,1);
J_tip  = zeros(N*6,ndof);

dof_start = 1; %starting dof of current rod
i_sig = 1;     %current computational point index

iLpre = Linkage.iLpre;
g_ini = Linkage.g_ini;

for i=1:N

    G = Linkage.G;
    if ~Linkage.Gravity
        G = G*0;
    elseif Linkage.UnderWater %G is reduced by a factor of 1-rho_water/rho_body
        G=G*(1-Linkage.Rho_water/Linkage.VLinks(Linkage.LinkIndex(i)).Rho);
    end

    if iLpre(i)>0
        g_here = g_tip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        J_here = J_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        J_here = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)))*J_here;
    else
        g_here    = g_ini((i-1)*4+1:i*4,:);
        J_here    = zeros(6,ndof);
    end

    %Joint
    %0 of joint
    
    g((i_sig-1)*4+1:i_sig*4,:) = g_here;
    J((i_sig-1)*6+1:i_sig*6,:) = J_here;
    i_sig                      = i_sig+1;

    dof_here = Linkage.CVRods{i}(1).dof;
    dofs_here = dof_start:dof_start+dof_here-1;

    q_here   = q(dofs_here);
    Phi_here = Linkage.CVRods{i}(1).Phi;
    xi_star  = Linkage.CVRods{i}(1).xi_star;
    
    S_here = zeros(6,ndof);
    if dof_here==0 %fixed joint (N)
        gstep    = eye(4);
    else
        xi = Phi_here*q_here+xi_star;
        [gstep,Tg] = variable_expmap_gTg(xi); %mex file is slightly slower for some reason
        S_here(:,dofs_here) = Tg*Phi_here;
    end

    %updating g, Jacobian
    g_here = g_here*gstep;
    J_here = dinamico_Adjoint(ginv(gstep))*(J_here+S_here);

    if ~Linkage.OneBasis %thinking about POD basis of Abdulaziz
        dof_start = dof_start+dof_here;
    end

    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'

        gi     = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
        g_here = g_here*gi;
        J_here = dinamico_Adjoint(ginv(gi))*J_here;

        g((i_sig-1)*4+1:i_sig*4,:) = g_here;
        J((i_sig-1)*6+1:i_sig*6,:) = J_here;
        i_sig                      = i_sig+1;

        M_here = Linkage.VLinks(Linkage.LinkIndex(i)).M;
        F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;

        % bringing all quantities to the end of rigid link
        gf     = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
        g_here = g_here*gf;
        J_here = dinamico_Adjoint(ginv(gf))*J_here;
    end

    for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1 %will run only if soft link

        dof_here  = Linkage.CVRods{i}(j+1).dof;
        dofs_here = dof_start:dof_start+dof_here-1;
        q_here    = q(dofs_here);
        
        gi      = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        ld      = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        xi_star = Linkage.CVRods{i}(j+1).xi_star;
        Ms      = Linkage.CVRods{i}(j+1).Ms;
        Xs      = Linkage.CVRods{i}(j+1).Xs;
        Ws      = Linkage.CVRods{i}(j+1).Ws;
        nip     = Linkage.CVRods{i}(j+1).nip;

        %updating g, Jacobian, Jacobian_dot and eta at X=0
        g_here        = g_here*gi;
        J_here        = dinamico_Adjoint(ginv(gi))*J_here;

        g((i_sig-1)*4+1:i_sig*4,:) = g_here;
        J((i_sig-1)*6+1:i_sig*6,:) = J_here;
        
        ii = 1;
        if Ws(ii)>0
            M_here = Ws(ii)*Ms(6*(ii-1)+1:6*ii,:);
            F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G;
        end
        i_sig  = i_sig+1;
        
        for ii=2:nip
            
            H = (Xs(ii)-Xs(ii-1))*ld;
            
            if Linkage.Z_order==4
                
                xi_Z1here = xi_star(6*(ii-2)+1:6*(ii-1),2);
                xi_Z2here = xi_star(6*(ii-2)+1:6*(ii-1),3);
                
                Phi_Z1here  = Linkage.CVRods{i}(j+1).Phi_Z1(6*(ii-2)+1:6*(ii-1),:);%note this step
                Phi_Z2here  = Linkage.CVRods{i}(j+1).Phi_Z2(6*(ii-2)+1:6*(ii-1),:);
                if dof_here>0
                    xi_Z1here = Phi_Z1here*q_here+xi_Z1here;
                    xi_Z2here = Phi_Z2here*q_here+xi_Z2here;
                end
                ad_xi_Z1here = dinamico_adj(xi_Z1here);
                Z_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                                 ((sqrt(3)*H^2)/12)*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here); 

                Omega_here = (H/2)*(xi_Z1here+xi_Z2here)+...
                               ((sqrt(3)*H^2)/12)*ad_xi_Z1here*xi_Z2here;
                      
            else % order 2
                xi_Zhere = xi_star(6*(ii-2)+1:6*(ii-1),4);

                Phi_Zhere  = Linkage.CVRods{i}(j+1).Phi_Z(6*(ii-2)+1:6*(ii-1),:);%note this step
                if dof_here>0
                    xi_Zhere = Phi_Zhere*q_here+xi_Zhere;
                end
                Z_here = H*Phi_Zhere;
                Omega_here  = H*xi_Zhere;

            end
                        
            [gstep,TOmega_here] = variable_expmap_gTg(Omega_here); %mex file is slightly slower

            S_here = zeros(6,ndof);
            S_here(:,dofs_here) = TOmega_here*Z_here;

            %updating g, Jacobian, Jacobian_dot and eta
            g_here = g_here*gstep;
            J_here = dinamico_Adjoint(ginv(gstep))*(J_here+S_here);

            g((i_sig-1)*4+1:i_sig*4,:) = g_here;
            J((i_sig-1)*6+1:i_sig*6,:) = J_here;
            i_sig = i_sig+1;

            %integrals evaluation
            if Ws(ii)>0
                M_here = Ws(ii)*Ms(6*(ii-1)+1:6*ii,:);
                F = F+J_here'*M_here*dinamico_Adjoint(ginv(g_here))*G; 
            end

        end

        %updating g, Jacobian, Jacobian_dot and eta at X=L
        gf     = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        g_here = g_here*gf;
        J_here = dinamico_Adjoint(ginv(gf))*J_here;

        if ~Linkage.OneBasis %thinking about POD basis of Abdulaziz
            dof_start = dof_start+dof_here;
        end
    end
    g_tip((i-1)*4+1:i*4,:) = g_here;
    J_tip((i-1)*6+1:i*6,:) = J_here;
end

%% Point Wrench 

for ip=1:Linkage.np
    Fp_here = Linkage.Fp_vec{ip}(0);
    i_sig = Linkage.Fp_sig(ip);

    if ~Linkage.LocalWrench(i) %if in the global frame
        g_here = g((i_sig-1)*4+1:i_sig*4,:);
        g_here(1:3,4) = zeros(3,1); %only rotational part
        Fp_here = (dinamico_Adjoint(g_here))'*Fp_here; %rotated into the local frame. Adj' = coAdj^-1
    end
    J_here = J((i_sig-1)*6+1:i_sig*6,:);
    F=F+J_here'*Fp_here;
end
%% Closed Chain Constraint

A = zeros(Linkage.CLprecompute.nCLp,ndof);
e = zeros(Linkage.CLprecompute.nCLp,1);

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
    else %if ground
        gA = Linkage.gACLj{iCLj};
        JA = zeros(6,ndof);
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
    else %if ground
        gB = Linkage.gBCLj{iCLj};
        JB = zeros(6,ndof);
    end
    
    %constraint force is in the B frame. Hence we need to bring all to B frame
    gBA = ginv(gB)*gA;
    OmegaBA = piecewise_logmap(gBA); %twist vector from B to A in R^6
    Adj_gBA = dinamico_Adjoint(gBA);
    diffJAB = Adj_gBA*JA-JB; %in B frame

    A(il_start:il_start+nl-1,:) = Phi_p'*diffJAB;
    e(il_start:il_start+nl-1) = Phi_p'*OmegaBA;
    
    il_start = il_start+nl;
end

F = F+A'*lambda;

%A should be FULL ROW RANK Matrix. Otherwise Problem!

% A possible fix below (needs more work)
% if rank(A)<size(A,1) %Have to take care of this
%     [~,iRows] = LIRows(A); 
%     e         = e(iRows); %iRows are the linearly independent rows of A. Comment this for some
% end

%% Custom External Force

if Linkage.CEF
    Fext  = CustomExtForce(Linkage,q,g,J); %should be point wrench in local frame and its derivatives
    for i_sig=1:Linkage.nsig
        F = F+J((i_sig-1)*6+1:i_sig*6,:)'*Fext;
    end
end

%% Actuation

tau = -Linkage.K*q; %tau = Bu-Kq

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
                            for ia =Linkage.i_sact{i}{j}
                                dc = Linkage.dc{ia,i}{j}(:,ii);
                                dcp = Linkage.dcp{ia,i}{j}(:,ii);
                                Phi_a(:,ia) = SoftActuator_FD(dc,dcp,xihat_123);
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
%% Custom Soft Actuation

if Linkage.CA
    Fact = CustomActuation(Linkage,q,g,J,0,zeros(ndof,1),zeros(6*nsig,1),zeros(6*nsig,ndof));
    tauC = CustomActuation_Qspace(Linkage,Fact,dFact_dq);
    tau = tau+tauC;
end


%% Equilibrium

taupF = tau+F;

if Linkage.nCLj>0 %if there are closed chain joints
    Res = [taupF;e];
else
    Res = taupF;
end

if staticsOptions.magnifier
    Res = Res*staticsOptions.magnifierValue;
end

end