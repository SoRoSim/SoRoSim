%Function that calculates the generalized actuation matrix (Bq)
%Inspired by dynamicsSolver and updated to match SorosimLinkage properties
function Bq = ActuationMatrix(Linkage,q)

    if isrow(q)
        q = q';
    end

    if Linkage.Actuated
        
        J    = Linkage.Jacobian(q);
        nact = Linkage.nact;
        ndof = Linkage.ndof;
        Bq   = zeros(ndof,nact);
        
        N_jact = Linkage.N_jact; % number of Links whose joints are actuated
        n_jact = Linkage.n_jact; % total number of joint actuators
        
        %% 1. Rigid/Joint Actuation
        % revolute, prismatic, helical joints (1D joints)
        Bj1   = Linkage.Bj1; 
        na_j1 = size(Bj1,2); 
        if na_j1 > 0
            Bq(:,1:na_j1) = Bj1;
        end
        
        % for other multi-DOF joints
        i_jact  = Linkage.i_jact;
        i_u     = na_j1 + 1;
        i_jactq = Linkage.i_jactq;
        
        for ii = na_j1+1:N_jact
            i = i_jact(ii);
            dof_here = Linkage.CVRods{i}(1).dof;
            us_here  = i_u : i_u+dof_here-1;
            dofs_here = i_jactq(us_here);
            
            if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype == 'C'
                Bq(dofs_here, us_here) = eye(2);
            else 
                % Utilizing ActuationPrecompute as done in dynamicsSolver
                i_sig = Linkage.ActuationPrecompute.i_sig_act(ii); 
                
                if Linkage.VLinks(Linkage.LinkIndex(i)).linktype == 'r'
                    gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
                else
                    gi = Linkage.VLinks(Linkage.LinkIndex(i)).gi{1};
                end
                
                AdgstepinvS_here = dinamico_Adjoint(gi) * J((i_sig-1)*6+1:i_sig*6, i_jactq(i_u:i_u+dof_here-1));
                Phi_here = Linkage.CVRods{i}(1).Phi;
                Bq(dofs_here, us_here) = AdgstepinvS_here' * Phi_here;
            end
            i_u = i_u + dof_here;
        end
        
        %% 2. Soft/Cable Actuation
        if Linkage.n_sact > 0
            dof_start = 1;
            for i = 1:Linkage.N
                
                dof_here = Linkage.CVRods{i}(1).dof;
                if ~Linkage.OneBasis
                    dof_start = dof_start + dof_here;
                end
                
                for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
                    na_here   = length(Linkage.i_sact{i}{j});
                    dof_here  = Linkage.CVRods{i}(j+1).dof;
                    dofs_here = dof_start : dof_start+dof_here-1;
                    
                    if ~Linkage.OneBasis
                        dof_start = dof_start + dof_here;
                    end
                    
                    if na_here > 0
                        Ws  = Linkage.CVRods{i}(j+1).Ws;
                        nip = Linkage.CVRods{i}(j+1).nip; % Defining nip locally!
                        B_here = zeros(dof_here, na_here);
                        
                        for ii = 1:nip
                            if Ws(ii) > 0
                                Phi = Linkage.CVRods{i}(j+1).Phi((ii-1)*6+1:ii*6,:);
                                xi  = Phi*q(dofs_here) + Linkage.CVRods{i}(j+1).xi_star((ii-1)*6+1:ii*6,1);
                                
                                Phi_a = zeros(6,na_here);
                                xihat_123 = [0 -xi(3) xi(2) xi(4); xi(3) 0 -xi(1) xi(5); -xi(2) xi(1) 0 xi(6)];
                                
                                ia_here = 1;
                                for ia = Linkage.i_sact{i}{j}
                                    dc  = Linkage.dc{ia,i}{j}(:,ii);
                                    dcp = Linkage.dcp{ia,i}{j}(:,ii);
                                    Phi_a(:,ia_here) = SoftActuator_FD(dc, dcp, xihat_123);
                                    ia_here = ia_here + 1;
                                end
                                B_here = B_here + Ws(ii) * Phi' * Phi_a;
                            end
                        end
                        % Map B_here into the full Bq matrix using n_jact offset
                        Bq(dofs_here, n_jact + Linkage.i_sact{i}{j}) = B_here;
                    end
                end
            end
        end
        
    else
        Bq = 0;
    end
end