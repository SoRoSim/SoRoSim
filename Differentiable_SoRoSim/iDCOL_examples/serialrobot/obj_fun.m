% ======================================================================
% Objective: smoothness + (optional) length
% ======================================================================
function [J, dJdx] = obj_fun(x,prob)
    N = prob.N;
    Popt = reshape(x,6,N);
    P = [prob.q_init, Popt, prob.q_des];   % 6 x (N+2)

    % Smoothness: sum ||p_{k+1}-2p_k+p_{k-1}||^2
    J_s = 0;
    dJs = zeros(size(P));

    for k = 2:N+1
        a = P(:,k+1) - 2*P(:,k) + P(:,k-1);

        J_s = J_s + a.'*a;

        dJs(:,k+1) = dJs(:,k+1) + 2*a;
        dJs(:,k)   = dJs(:,k)   - 4*a;
        dJs(:,k-1) = dJs(:,k-1) + 2*a;
    end

    % Length: sum ||p_{k+1}-p_k||^2
    J_l = 0;
    dJl = zeros(size(P));

    for k = 1:N+1
        b = P(:,k+1) - P(:,k);

        J_l = J_l + b.'*b;

        dJl(:,k+1) = dJl(:,k+1) + 2*b;
        dJl(:,k)   = dJl(:,k)   - 2*b;
    end

    J = prob.w_smooth*J_s + prob.w_length*J_l;

    dJ_full = prob.w_smooth*dJs + prob.w_length*dJl;

    % Keep only gradients wrt optimization variables
    dJdx = dJ_full(:,2:N+1);
    dJdx = dJdx(:);
end

