function qdqdd = derivatives(Linkage,t,qqd,dynamicAction,GUI_actionInput,dynamicsOptions) %u is a function of t
    
    persistent tlast
    if dynamicsOptions.displayTime
        if t==0
            tlast=cputime;
        end
        if cputime-tlast>0.5
 		        tlast = cputime;
                disp(t);
        end
    end

    ndof = Linkage.ndof;

    q  = qqd(1:ndof);
    qd = qqd(ndof+1:2*ndof);

    if Linkage.Actuated&&(~Linkage.CAI)

        if GUI_actionInput %if action input is from GUI

            nact = Linkage.nact;
            n_k  = Linkage.ActuationPrecompute.n_k;
    
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
    
    y = dynamicsSolver(Linkage,t,q,qd,action); %solves for x = [qdd_u; u_u; lambda], for a given t, q, qd, and action
    qdd = y(1:ndof);

    if Linkage.Actuated
        if n_k>0
            qdd(Linkage.ActuationPrecompute.index_q_u) = y(1:ndof-n_k);
            qdd(Linkage.ActuationPrecompute.index_q_k) = action(nact-n_k+1:end);
        end
    end
    
    qdqdd = [qd;qdd];

end