function dxd_dx = ODEJacobian(Linkage,t,x,dynamicAction,GUI_actionInput)
    
    ndof = Linkage.ndof;

    q  = x(1:ndof);
    qd = x(ndof+1:2*ndof);

    if Linkage.Actuated&&(~Linkage.CAI)

        nact = Linkage.nact;
        n_k  = Linkage.ActuationPrecompute.n_k;

        if GUI_actionInput %if action input is from GUI
    
            action = zeros(nact,1);
            q_k = zeros(n_k,1);
            qd_k = zeros(n_k,1);
    
            for ia = 1:nact
                if ia<=nact-n_k
                    action(ia) = dynamicAction{ia}(t);
                else
                    action(ia) = dynamicAction{ia}{3}(t);
                    q_k(ia-nact+n_k)  = dynamicAction{ia}{1}(t);
                    qd_k(ia-nact+n_k) = dynamicAction{ia}{2}(t);
                end
            end

        else %if action input is from a file
            [action,q_k,qd_k] = dynamicAction(t);
        end

        q(Linkage.ActuationPrecompute.index_q_k)  = q_k;
        qd(Linkage.ActuationPrecompute.index_q_k) = qd_k;
    else
        action = [];
    end

    [y,C,B_action,action] = dynamicsSolver(Linkage,t,[q;qd],action); %solves for x = [qdd_u; u_u; lambda], for a given t, q, qd, and action
    qdd = y(1:ndof);
    u   = action;
    lambda = y(ndof+1:end);

    if Linkage.Actuated
        n_k  = Linkage.ActuationPrecompute.n_k;

        if n_k>0
            qdd(Linkage.ActuationPrecompute.index_q_u) = y(1:ndof-n_k);
            qdd(Linkage.ActuationPrecompute.index_q_k) = action(nact-n_k+1:end);
            u(Linkage.ActuationPrecompute.index_u_k) = action(1:nact-n_k);
            u(Linkage.ActuationPrecompute.index_u_u) = y(ndof-n_k+1:ndof);
        end
    end

    %[dID_dq,dID_dqd,dtau_dq,dtau_dqd,de_dq,de_dqd,daction_dq,daction_dqd] = DAEJacobians_qqd_i1(Linkage,q,qd,qdd,u,lambda)
    [dID_dq,dID_dqd,dtau_dq,dtau_dqd,de_dq,de_dqd,daction_dq,daction_dqd] = DAEi1Jacobians_qqd(Linkage,t,q,qd,qdd,u,lambda);
  
    dFD_dq = zeros(ndof,ndof);
    dFD_dqd = zeros(ndof,ndof);

    if Linkage.nCLj>0 %if there are closed chain joints
    
        if Linkage.Actuated
            if Linkage.CAI
                dy_dq  = C\([dtau_dq-dID_dq;-de_dq]+B_action*daction_dq);
                dy_dqd = C\([dtau_dqd-dID_dqd;-de_dqd]+B_action*daction_dqd);
            else
                dy_dq  = C\([dtau_dq-dID_dq;-de_dq]);
                dy_dqd = C\([dtau_dqd-dID_dqd;-de_dqd]);
            end

            dFD_dq(Linkage.ActuationPrecompute.index_q_u,Linkage.ActuationPrecompute.index_q_u) = dy_dq(1:ndof-n_k,Linkage.ActuationPrecompute.index_q_u);
            dFD_dqd(Linkage.ActuationPrecompute.index_q_u,Linkage.ActuationPrecompute.index_q_u) = dy_dqd(1:ndof-n_k,Linkage.ActuationPrecompute.index_q_u);

        else
            dy_dq  = C\([dtau_dq-dID_dq;-de_dq]);
            dy_dqd = C\([dtau_dqd-dID_dqd;-de_dqd]);

            dFD_dq = dy_dq(1:ndof,1:ndof);
            dFD_dqd = dy_dqd(1:ndof,1:ndof);
        end
    else
    
        if Linkage.Actuated
            if Linkage.CAI
                dy_dq  = C\(dtau_dq-dID_dq+B_action*daction_dq);
                dy_dqd = C\(dtau_dqd-dID_dqd+B_action*daction_dqd);
            else
                dy_dq  = C\(dtau_dq-dID_dq);
                dy_dqd = C\(dtau_dqd-dID_dqd);
            end

            dFD_dq(Linkage.ActuationPrecompute.index_q_u,Linkage.ActuationPrecompute.index_q_u) = dy_dq(1:ndof-n_k,Linkage.ActuationPrecompute.index_q_u);
            dFD_dqd(Linkage.ActuationPrecompute.index_q_u,Linkage.ActuationPrecompute.index_q_u) = dy_dqd(1:ndof-n_k,Linkage.ActuationPrecompute.index_q_u);

        else
            dy_dq  = C\(dtau_dq-dID_dq);
            dy_dqd = C\(dtau_dqd-dID_dqd);

            dFD_dq = dy_dq;
            dFD_dqd = dy_dqd;
        end
    
    end
    
    dxd_dx = zeros(2*ndof,2*ndof);
    dxd_dx(1:ndof,ndof+1:2*ndof) = eye(ndof);

    if Linkage.Actuated&&(~Linkage.CAI)
        dxd_dx(1:ndof,ndof+Linkage.ActuationPrecompute.index_q_k) = zeros(ndof,n_k);
    end
    
    dxd_dx(ndof+1:2*ndof,:) = [dFD_dq,dFD_dqd];

end