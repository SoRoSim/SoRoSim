% ======================================================================
% Objective (NO GRADIENT): smoothness + (optional) length
% Returns only J so fmincon finite-differences if needed.
% ======================================================================
function J = obj_fun_nojac(x, prob)

    N = prob.N;
    P = reshape(x, 3, N);

    % Smoothness: sum ||p_{k+1}-2p_k+p_{k-1}||^2
    J_s = 0;
    for k = 2:N-1
        a = P(:,k+1) - 2*P(:,k) + P(:,k-1);
        J_s = J_s + a.'*a;
    end

    % Length: sum ||p_{k+1}-p_k||^2
    J_l = 0;
    for k = 1:N-1
        b = P(:,k+1) - P(:,k);
        J_l = J_l + b.'*b;
    end

    J = prob.w_smooth*J_s + prob.w_length*J_l;
end
