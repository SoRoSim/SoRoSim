%Function that calculates the derivative of the jacobian with respect to
%time (Jd) at every significant point (24.05.2021)

function Jd = Jacobiandot(Linkage,q,qd,i_here,j_here) %i_here is link index, j_here is division (0 for joint)

if isrow(q)
    q=q';
end
if isrow(qd)
    qd=qd';
end

N       = Linkage.N;
ndof    = Linkage.ndof;
g_ini   = Linkage.g_ini; %initial configuration of all link wrt its previous link
iLpre   = Linkage.iLpre;
g_tip   = repmat(eye(4),N,1);
Jd_tip  = zeros(6*N,ndof);
eta_tip = zeros(6*N,1);

full = false;
if nargin==5
    if j_here==0
        nsig = 1;
    else
        nsig = Linkage.CVTwists{i_here}(j_here+1).nip; %j_here>1 is allowed only for soft links
    end
elseif nargin==4
    if Linkage.VLinks(Linkage.LinkIndex(i_here)).linktype=='s'
        nsig = 1;
        for j=1:Linkage.VLinks(Linkage.LinkIndex(i_here)).npie-1
            nsig = nsig+Linkage.CVTwists{i_here}(j+1).nip;
        end
    else
        nsig = 2; %joint and CM
    end
else
    full  = true;
    nsig  = Linkage.nsig;
end

Jd        = zeros(6*nsig,ndof);
dof_start = 1; %starting dof of current piece
i_sig     = 1;


for i = 1:N
    
    if iLpre(i)>0
        g_here       = g_tip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        Ad_g_ini_inv = dinamico_Adjoint(ginv(g_ini((i-1)*4+1:i*4,:)));
        Jd_here      = Ad_g_ini_inv*Jd_tip((iLpre(i)-1)*6+1:iLpre(i)*6,:);
        eta_here     = Ad_g_ini_inv*eta_tip((iLpre(i)-1)*6+1:iLpre(i)*6);
    else
        g_here   = g_ini((i-1)*4+1:i*4,:);
        Jd_here  = zeros(6,ndof);
        eta_here   = zeros(6,1);
    end
    
    %Joint
    dof_here = Linkage.CVTwists{i}(1).dof;
    q_here   = q(dof_start:dof_start+dof_here-1);
    qd_here  = qd(dof_start:dof_start+dof_here-1);
    Phi_here = Linkage.CVTwists{i}(1).Phi;
    xi_star  = Linkage.CVTwists{i}(1).xi_star;
    
    if dof_here == 0 %fixed joint (N)
        g_joint = eye(4);
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
    Jd_here        = Ad_g_joint_inv*(Jd_here+Sd_here);
    eta_here       = Ad_g_joint_inv*(eta_here+S_here(:,dof_start:dof_start+dof_here-1)*qd_here);
    
    if full||(i==i_here&&nargin==4)||(i==i_here&&j_here==0)
        Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
        i_sig                       = i_sig+1;
    end
    
    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype == 'r'
        
        gi        = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
        g_here    = g_here*gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        
        if full||(i==i_here&&nargin==4)
            Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
            i_sig                       = i_sig+1;
        end
        
        % bringing all quantities to the end of rigid link
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        
    end
    
    dof_start = dof_start+dof_here;
    
    for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1 %will run only if soft link
        
        dof_here = Linkage.CVTwists{i}(j+1).dof;
        q_here   = q(dof_start:dof_start+dof_here-1);
        qd_here  = qd(dof_start:dof_start+dof_here-1);
        xi_star  = Linkage.CVTwists{i}(j+1).xi_star;
        gi       = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        ld       = Linkage.VLinks(Linkage.LinkIndex(i)).lp{j};
        Xs       = Linkage.CVTwists{i}(j+1).Xs;
        nip      = Linkage.CVTwists{i}(j+1).nip;
        
        
        %updating g, Jacobian, Jacobian_dot and eta at X=0
        g_here   = g_here*gi;
        Ad_gi_inv = dinamico_Adjoint(ginv(gi));
        Jd_here   = Ad_gi_inv*Jd_here;
        eta_here  = Ad_gi_inv*eta_here;
        
        if full||(i==i_here&&nargin==4)||(i==i_here&&j==j_here)
            Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
            i_sig                       = i_sig+1;
        end 
        
        
        for ii = 2:nip
            
            H    = (Xs(ii)-Xs(ii-1))*ld;
            
            if Linkage.Z_order==4
                
                xi_Z1here = xi_star(6*(ii-2)+1:6*(ii-1),2); 
                xi_Z2here = xi_star(6*(ii-2)+1:6*(ii-1),3);
                    
                Phi_Z1here  = Linkage.CVTwists{i}(j+1).Phi_Z1(6*(ii-2)+1:6*(ii-1),:);%note this step
                Phi_Z2here  = Linkage.CVTwists{i}(j+1).Phi_Z2(6*(ii-2)+1:6*(ii-1),:);

                if dof_here>0
                    xi_Z1here = Phi_Z1here*q_here+xi_Z1here;
                    xi_Z2here = Phi_Z2here*q_here+xi_Z2here;

                    xid_Z1here  = Phi_Z1here*qd_here;
                    xid_Z2here  = Phi_Z2here*qd_here;

                    ad_xi_Z1here = dinamico_adj(xi_Z1here);

                    Z_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                                       ((sqrt(3)*H^2)/12)*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);

                    Zd_here = ((sqrt(3)*H^2)/12)*(dinamico_adj(xid_Z1here)*Phi_Z2here-dinamico_adj(xid_Z2here)*Phi_Z1here); 

                    Omegad_here   = Z_here*qd_here;
                else
                    ad_xi_Z1here = dinamico_adj(xi_Z1here);
                    Z_here  = (H/2)*(Phi_Z1here+Phi_Z2here)+...
                                       ((sqrt(3)*H^2)/12)*(ad_xi_Z1here*Phi_Z2here-dinamico_adj(xi_Z2here)*Phi_Z1here);
                    Zd_here = zeros(6,dof_here); 
                    Omegad_here = zeros(6,1); 
                end

                Omega_here = (H/2)*(xi_Z1here+xi_Z2here)+...
                             ((sqrt(3)*H^2)/12)*ad_xi_Z1here*xi_Z2here;
                      
            else % order 2
                
                xi_Zhere = xi_star(6*(ii-2)+1:6*(ii-1),4);
                    
                Phi_Zhere  = Linkage.CVTwists{i}(j+1).Phi_Z(6*(ii-2)+1:6*(ii-1),:);%note this step

                if dof_here>0
                    xi_Zhere = Phi_Zhere*q_here+xi_Zhere;

                    Z_here = H*Phi_Zhere;

                    Omegad_here     = Z_here*qd_here;
                else
                    Z_here = H*Phi_Zhere;
                    Omegad_here   = zeros(6,1); 
                end

                
                Omega_here  = H*xi_Zhere;

            end
            
            [gh,Tg,Tgd] = variable_expmap_gTgTgd_mex(Omega_here,Omegad_here); % mex code, C program
            
            S_here                                    = zeros(6,ndof);
            S_here(:,dof_start:dof_start+dof_here-1)  = Tg*Z_here;
            Sd_here                                   = zeros(6,ndof);
            Sd_here(:,dof_start:dof_start+dof_here-1) = dinamico_adj(eta_here)*S_here(:,dof_start:dof_start+dof_here-1)+Tgd*Z_here;
            
            if Linkage.Z_order==4
                Sd_here(:,dof_start:dof_start+dof_here-1) = Sd_here(:,dof_start:dof_start+dof_here-1)+Tg*Zd_here;
            end
            
            %updating g, Jacobian, Jacobian_dot and eta
            g_here    = g_here*gh;
            Ad_gh_inv = dinamico_Adjoint(ginv(gh));
            Jd_here   = Ad_gh_inv*(Jd_here+Sd_here); %full
            eta_here  = Ad_gh_inv*(eta_here+S_here(:,dof_start:dof_start+dof_here-1)*qd_here);
            
            
            if full||(i==i_here&&nargin==4)||(i==i_here&&j==j_here)
                Jd((i_sig-1)*6+1:i_sig*6,:) = Jd_here;
                i_sig                       = i_sig+1;
            end
            
        end
        
        %updating g, Jacobian, Jacobian_dot and eta at X=L
        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        g_here    = g_here*gf;
        Ad_gf_inv = dinamico_Adjoint(ginv(gf));
        Jd_here   = Ad_gf_inv*Jd_here;
        eta_here  = Ad_gf_inv*eta_here;
        
        dof_start  = dof_start+dof_here;
    end
    g_tip((i-1)*4+1:i*4,:)   = g_here;
    Jd_tip((i-1)*6+1:i*6,:)  = Jd_here;
    eta_tip((i-1)*6+1:i*6,:) = eta_here;
end
end

