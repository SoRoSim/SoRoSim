function qd_free_next = iEulerStep(Linkage,q,qd,dt,u)

qd_free_next = qd;

tol_abs  = 1e-8;
tol_rel  = 1e-6;
max_iter = 10;

jac_update_every = 1; %see improvement later
stagnation_ratio = 0.8;

R_prev_norm = inf;

for iter = 1:max_iter

    update_J = iter == 1 || mod(iter-1,jac_update_every) == 0;

    if update_J
        [R, JR] = iEulerResidueJacobian(qd_free_next, Linkage, q, qd, dt, u);
        dJR = decomposition(JR, 'lu');
    else
        R = iEulerResidueJacobian(qd_free_next, Linkage, q, qd, dt, u);
    end

    R_norm = norm(R);

    if iter == 1
        R0_norm = R_norm;
    end

    if R_norm < tol_abs + tol_rel*R0_norm
        return;
    end

    % If reused Jacobian is stale, refresh it immediately
    if iter > 1 && R_norm > stagnation_ratio*R_prev_norm
        [R, JR] = iEulerResidueJacobian(qd_free_next, Linkage, q, qd, dt, u);
        dJR = decomposition(JR, 'lu');
        R_norm = norm(R);
    end

    delta = dJR \ R;
    qd_free_next = qd_free_next - delta;

    R_prev_norm = R_norm;
end

error('Maximum iterations reached without convergence.');

end