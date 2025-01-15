%Function that calculates the forward kinematics of the linkage from the base to the linkage tip at every significant points
%Last modified by Anup Teejo Mathew 29.11.2024
function g = FwdKinematics(Linkage,q,i_here,j_here) %i_here is link index, j_here is division (0 for joint)

full = false;
if nargin==4
    if j_here==0
        nsig = 1;
    else
        nsig = Linkage.CVRods{i_here}(j_here+1).nip;
    end
elseif nargin==3
    if Linkage.VLinks(Linkage.LinkIndex(i_here)).linktype=='s'
        nsig = 1;
        for j=1:Linkage.VLinks(Linkage.LinkIndex(i_here)).npie-1
            nsig = nsig+Linkage.CVRods{i_here}(j+1).nip;
        end
    else
        nsig = 2; %joint and CM
    end
else
    full  = true;
    nsig  = Linkage.nsig;
end
g     = zeros(4*nsig,4); 
i_sig = 1;

if isrow(q)
    q=q';
end

N         = Linkage.N;
g_ini     = Linkage.g_ini;
g_Ltip    = repmat(eye(4),N,1);
iLpre     = Linkage.iLpre;

dof_start = 1;                         %starting dof of current piece


for i = 1:N
    
    if Linkage.iLpre(i)>0
        g_here=g_Ltip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
    else
        g_here=g_ini((i-1)*4+1:i*4,:);
    end
    
    %Joint
    dof_here = Linkage.CVRods{i}(1).dof;
    q_here   = q(dof_start:dof_start+dof_here-1);
    Phi_here   = Linkage.CVRods{i}(1).Phi;
    xi_star  = Linkage.CVRods{i}(1).xi_star;
    
    if dof_here == 0                   %fixed joint (N)
        g_joint  = eye(4);
    else
        xi      = Phi_here*q_here+xi_star;
        g_joint = variable_expmap_g(xi);
    end

    g_here                     = g_here*g_joint;
    if full||(i==i_here&&nargin==3)||(i==i_here&&j_here==0)
        g((i_sig-1)*4+1:i_sig*4,:) = g_here;
        i_sig                      = i_sig+1;
    end
    
    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype == 'r'
        
        gi                         = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
        g_here                     = g_here*gi;
        if full||(i==i_here&&nargin==3)
            g((i_sig-1)*4+1:i_sig*4,:) = g_here;
            i_sig                      = i_sig+1;
        end
        % bringing all quantities to the end of rigid link
        gf     = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
        g_here = g_here*gf;
    end
    
    dof_start = dof_start+dof_here;
    
    for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
        
        dof_here = Linkage.CVRods{i}(j+1).dof;
        q_here   = q(dof_start:dof_start+dof_here-1);
        xi_star  = Linkage.CVRods{i}(j+1).xi_star;
        gi       = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        ld       = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        Xs       = Linkage.CVRods{i}(j+1).Xs;
        nip      = Linkage.CVRods{i}(j+1).nip;
        
        if Linkage.Z_order==4
            Phi_Z1       = Linkage.CVRods{i}(j+1).Phi_Z1;
            Phi_Z2       = Linkage.CVRods{i}(j+1).Phi_Z2;
        else %order 2
            Phi_Z        = Linkage.CVRods{i}(j+1).Phi_Z;
        end
        
        %updating g, Jacobian, Jacobian_dot and eta at X=0
        g_here = g_here*gi;
        if full||(i==i_here&&nargin==3)||(i==i_here&&j==j_here)
            g((i_sig-1)*4+1:i_sig*4,:) = g_here;
            i_sig                      = i_sig+1;
        end
        
        for ii = 2:nip

            H    = (Xs(ii)-Xs(ii-1))*ld;
            
            if Linkage.Z_order==4
                
                xi_Z1here = xi_star(6*(ii-2)+1:6*(ii-1),2);
                xi_Z2here = xi_star(6*(ii-2)+1:6*(ii-1),3);
                
                if dof_here>0
                    if dof_here>0
                        Phi_Z1here  = Phi_Z1(6*(ii-2)+1:6*(ii-1),:);%note this step
                        Phi_Z2here  = Phi_Z2(6*(ii-2)+1:6*(ii-1),:);
                        xi_Z1here = Phi_Z1here*q_here+xi_Z1here;
                        xi_Z2here = Phi_Z2here*q_here+xi_Z2here;
                    end
                end
                ad_xi_Z1here = dinamico_adj(xi_Z1here);  
                Gamma_here   = (H/2)*(xi_Z1here+xi_Z2here)+...
                                ((sqrt(3)*H^2)/12)*ad_xi_Z1here*xi_Z2here;
                      
            else % order 2
                xi_Zhere = xi_star(6*(ii-2)+1:6*(ii-1),4);
                
                if dof_here>0
                    Phi_Zhere  = Phi_Z(6*(ii-2)+1:6*(ii-1),:);%note this step
                    if dof_here>0
                        xi_Zhere = Phi_Zhere*q_here+xi_Zhere;
                    end
                end
                
                Gamma_here  = H*xi_Zhere;

            end

            gh              = variable_expmap_g(Gamma_here);
            
            g_here                     = g_here*gh;
            if full||(i==i_here&&nargin==3)||(i==i_here&&j==j_here)
                g((i_sig-1)*4+1:i_sig*4,:) = g_here;
                i_sig                      = i_sig+1;
            end

        end

        gf        = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
        g_here    = g_here*gf;
        dof_start = dof_start+dof_here;
        
    end
    g_Ltip((i-1)*4+1:i*4,:) = g_here; 
end

end


