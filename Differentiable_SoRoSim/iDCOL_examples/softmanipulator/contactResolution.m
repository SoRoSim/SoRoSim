function [qd_next, gamma_n0_out] = contactResolution(Linkage, M, g, J, qd_free, contactInfo, gamma_n0, dt)

ndof = Linkage.ndof;
ncp  = Linkage.ncp;

k_c   = 1e4;
d_c   = 0.1;
mu_c  = 0.5;
eps_s = 1e-4;

if Linkage.Damped
    D = Linkage.D;
else
    D = zeros(ndof, ndof);
end

A = M + dt*D + dt^2*Linkage.K;

activePairs = contactInfo.activePairs;
iactive     = find(activePairs);
nac         = length(iactive);

if nac == 0
    qd_next = qd_free;
    gamma_n0_out = zeros(ncp, 1);
    return;
end

% Build stacked contact Jacobian and contact data
Jc      = zeros(3*nac, ndof);
n_stack = zeros(3, nac);
delta0  = zeros(nac, 1);
gn0     = zeros(nac, 1);

for k_idx = 1:nac
    icp    = iactive(k_idx);
    i_sigA = Linkage.Pairs(icp).body1.i_sig;
    i_sigB = Linkage.Pairs(icp).body2.i_sig;

    J_A = J((i_sigA-1)*6+1:i_sigA*6, :);
    g_A = g((i_sigA-1)*4+1:i_sigA*4, :);

    x_A = contactInfo.x(:, icp);      % contact point in A frame
    n_A = contactInfo.n(:, icp);      % normal from A to B, in A frame

    n_A = n_A / norm(n_A);

    if i_sigB > 0
        J_B   = J((i_sigB-1)*6+1:i_sigB*6, :);
        g_B   = g((i_sigB-1)*4+1:i_sigB*4, :);

        g_AB  = ginv(g_A)*g_B;

        % Relative body twist of A wrt B, expressed in A frame
        J_rel = J_A - dinamico_Adjoint(g_AB)*J_B;
    else
        J_rel = J_A;
    end

    % Point velocity Jacobian.
    % vc = Jc_i * qd = velocity of A wrt B at contact point, in A frame.
    P    = [dinamico_tilde(x_A), eye(3)];
    Jc_i = P * J_rel;

    idx = 3*(k_idx-1)+1:3*k_idx;

    Jc(idx, :)        = Jc_i;
    n_stack(:, k_idx) = n_A;
    delta0(k_idx)     = contactInfo.delta(icp);
    gn0(k_idx)        = gamma_n0(icp);
end

tol     = 1e-6;
maxiter = 20;
qd_next = qd_free;

for iter = 1:maxiter

    [r, H] = residualAndJacobian(qd_next);

    if norm(r) < tol
        break;
    end

    delta_qd = H \ r;

    % Armijo line search on residual norm
    alpha = 1.0;
    r_norm = norm(r);

    for ls = 1:20
        qd_trial = qd_next - alpha*delta_qd;

        r_trial = residualOnly(qd_trial);

        if norm(r_trial) < (1 - 1e-4*alpha)*r_norm
            break;
        end

        alpha = 0.5*alpha;
    end

    qd_next = qd_trial;
end

r_final = residualOnly(qd_next);

if iter == maxiter && norm(r_final) >= tol
    warning('contactResolution: Newton did not converge. Final norm(r)=%.2e', norm(r_final));
end

% Store positive scalar normal impulse magnitudes for next time step
gamma_n0_out = zeros(ncp, 1);

for k_idx = 1:nac
    icp = iactive(k_idx);
    idx = 3*(k_idx-1)+1:3*k_idx;

    vc_i = Jc(idx,:) * qd_next;
    n_i  = n_stack(:, k_idx);

    vn_i = dot(vc_i, n_i);

    delta_impl = delta0(k_idx) + dt*vn_i;
    hc         = 1 + d_c*vn_i;

    if delta_impl > 0 && hc > 0
        gamma_n0_out(icp) = dt * k_c * delta_impl * hc;
    else
        gamma_n0_out(icp) = 0;
    end
end

% ============================================================
% Nested helper: residual and Newton matrix
% ============================================================
function [r, H] = residualAndJacobian(qd)

    vc = Jc * qd;

    gamma = zeros(3*nac, 1);
    G     = zeros(3*nac, 3*nac);

    for kk = 1:nac
        idx = 3*(kk-1)+1:3*kk;

        vc_i     = vc(idx);
        n_i      = n_stack(:, kk);
        delta_i  = delta0(kk);
        gn_lag_i = gn0(kk);

        % vn > 0 means approaching because n points from A to B
        vn_i = dot(vc_i, n_i);

        Pt   = eye(3) - n_i*n_i';
        vt_i = Pt * vc_i;

        vt_soft = sqrt(vt_i.'*vt_i + eps_s^2);
        t_hat_s = vt_i / vt_soft;

        % implicit penetration
        delta_impl = delta_i + dt*vn_i;

        % Hunt-Crossley-like normal factor.
        % Since vn > 0 means approaching, damping should increase force.
        hc = 1 + d_c*vn_i;

        if delta_impl > 0 && hc > 0
            fn_i      = k_c * delta_impl * hc;
            gamma_n_i = dt * fn_i;

            dfn_dvn = k_c * (dt*hc + d_c*delta_impl);
            dgn_dvn = dt * dfn_dvn;
        else
            gamma_n_i = 0;
            dgn_dvn   = 0;
        end

        % Normal impulse on A is opposite n because n points from A to B
        gamma_normal_i = -gamma_n_i * n_i;

        % Lagged Coulomb friction
        gamma_t_i = -mu_c * gn_lag_i * t_hat_s;

        gamma(idx) = gamma_normal_i + gamma_t_i;

        % d gamma_normal / d vc
        G_normal_i = -dgn_dvn * (n_i*n_i');

        % d gamma_t / d vc
        %
        % t_hat_s = vt / sqrt(vt'vt + eps_s^2)
        % vt      = Pt * vc
        %
        Dt_hat = (eye(3) - t_hat_s*t_hat_s.') / vt_soft;
        G_t_i  = -mu_c * gn_lag_i * Dt_hat * Pt;

        G(idx,idx) = G_normal_i + G_t_i;
    end

    r = A*(qd - qd_free) - Jc.'*gamma;

    % Since G = dgamma/dvc, vc = Jc*qd:
    %
    % dr/dqd = A - Jc' * G * Jc
    H = A - Jc.'*G*Jc;
end

% ============================================================
% Nested helper: residual only
% ============================================================
function r = residualOnly(qd)

    vc = Jc * qd;

    gamma = zeros(3*nac, 1);

    for kk = 1:nac
        idx = 3*(kk-1)+1:3*kk;

        vc_i     = vc(idx);
        n_i      = n_stack(:, kk);
        delta_i  = delta0(kk);
        gn_lag_i = gn0(kk);

        vn_i = dot(vc_i, n_i);

        Pt   = eye(3) - n_i*n_i';
        vt_i = Pt * vc_i;

        vt_soft = sqrt(vt_i.'*vt_i + eps_s^2);
        t_hat_s = vt_i / vt_soft;

        delta_impl = delta_i + dt*vn_i;
        hc         = 1 + d_c*vn_i;

        if delta_impl > 0 && hc > 0
            fn_i      = k_c * delta_impl * hc;
            gamma_n_i = dt * fn_i;
        else
            gamma_n_i = 0;
        end

        gamma_normal_i = -gamma_n_i * n_i;
        gamma_t_i      = -mu_c * gn_lag_i * t_hat_s;

        gamma(idx) = gamma_normal_i + gamma_t_i;
    end

    r = A*(qd - qd_free) - Jc.'*gamma;
end

end