%Function to display and save the actuation cables 
%Last modified by Anup Teejo Mathew (14.06.2021)
function CableDisplay(Linkage,dc_fn,iL,div_start,div_end)

%Forward kinematics
q         = zeros(Linkage.ndof,1);
N         = Linkage.N;
dof_start = 1;
g_ini     = Linkage.g_ini;
g_Ltip    = repmat(eye(4),N,1);
iLpre     = Linkage.iLpre;
VLinks    = Linkage.VLinks;
CVRods    = Linkage.CVRods;
LinkIndex = Linkage.LinkIndex;

xC        = [];
yC        = [];
zC        = [];

for i = 1:iL %upto iL is enough
    
    if iLpre(i)>0
        g_here = g_Ltip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
    else
        g_here = g_ini((i-1)*4+1:i*4,:);
    end
    
    VRods = CVRods{i};
    
    %Joints
    xi_star  = VRods(1).xi_star;
    Phi_here = VRods(1).Phi;
    dof_here = VRods(1).dof;
    q_here   = q(dof_start:dof_start+dof_here-1);
    
    if dof_here == 0 %fixed joint (N)
        g_joint = eye(4);
    else
        xi         = Phi_here*q_here+xi_star;
        g_joint    = variable_expmap_g(xi);
    end
    
    g_here = g_here*g_joint;

    if VLinks(LinkIndex(i)).linktype=='r'
        g_here  = g_here*VLinks(LinkIndex(i)).gi*VLinks(LinkIndex(i)).gf;
    end
    
    dof_start = dof_start+dof_here;
    %=============================================================================
    %Soft pieces
    for j = 1:(VLinks(LinkIndex(i)).npie)-1
        
        xi_starfn = VRods(j+1).xi_starfn;
        gi      = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
        ld  = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
        Phi_Scale = diag([1/ld 1/ld 1/ld 1 1 1]);

        Type    = Linkage.CVRods{i}(j+1).Type;

        Phi_dof      = Linkage.CVRods{i}(j+1).Phi_dof;
        Phi_odr      = Linkage.CVRods{i}(j+1).Phi_odr;

        Phi_h     = Linkage.CVRods{i}(j+1).Phi_h;
        
        %updating g, Jacobian, Jacobian_dot and eta at X=0
        g_here    = g_here*gi;
        dof_here  = VRods(j+1).dof;
        q_here    = q(dof_start:dof_start+dof_here-1);

        n_l      = VLinks(LinkIndex(i)).n_l;
        if i==iL&&j>=div_start&&j<=div_end
            n_l=n_l*4; %more points
        end
        
        Xs = linspace(0,1,n_l);
        H  = Xs(2)-Xs(1);
        Z  = (1/2)*H;          % Zanna quadrature coefficient
        
        if j==div_start
            x_c=0;
        end
        
        if i==iL&&j==div_start&&j<=div_end
            dc_here   = dc_fn(x_c);
            PosC_here = g_here*[dc_here;1];
            xC        = [xC PosC_here(1)];
            yC        = [yC PosC_here(2)];
            zC        = [zC PosC_here(3)];
        end
        
        for ii = 1:n_l-1
            
            X   = Xs(ii);
            X_Z = X+Z;
            
            xi_Zhere  = xi_starfn(X_Z);
            
            if ~isempty(q_here)                 
                if strcmp(Type,'FEM Like')
                    SubClass  = Linkage.CVRods{i}(j+1).SubClass;
                    xi_Zhere  = xi_Zhere+Phi_Scale*Phi_h(X_Z,Phi_dof,Phi_odr,SubClass)*q_here;
                elseif strcmp(Type,'Custom Basis')
                    xi_Zhere  = xi_Zhere+Phi_Scale*Phi_h(X_Z)*q_here;
                else
                    xi_Zhere  = xi_Zhere+Phi_Scale*Phi_h(X_Z,Phi_dof,Phi_odr)*q_here;
                end
            end
            Gamma_here = H*ld*xi_Zhere;
            
            gh         = variable_expmap_g(Gamma_here);
            g_here     = g_here*gh;
            
            if i==iL&&j>=div_start&&j<=div_end
                x_c       = x_c+H*ld;
                dc_here   = dc_fn(x_c);
                PosC_here = g_here*[dc_here;1];
                xC        = [xC PosC_here(1)];
                yC        = [yC PosC_here(2)];
                zC        = [zC PosC_here(3)];
            end

        end
        dof_start  = dof_start+dof_here;
    end
    g_Ltip((i-1)*4+1:i*4,:) = g_here; 
end

plot3(xC,yC,zC,'LineWidth',2,'Color','m');

end