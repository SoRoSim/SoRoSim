function [c, ceq, dc, dceq] = nl_con(x, prob)
    N     = prob.N;
    robot = prob.Linkage;
    Pairs = prob.Pairs;
    ncp   = numel(Pairs);

    P    = reshape(x, 6, N);
    ceq  = [];
    dceq = [];

    c  = zeros((ncp+1)*N, 1);
    dc = zeros(6*N, (ncp+1)*N);

    % --- z-floor constraints: -z <= 0  -------------------------------
    for i = 1:N
        g = getTransform(robot, P(:,i), "ee_link");
        J = geometricJacobian(robot, P(:,i), "ee_link");  % 6x6, [w; v] in base
        c(i) = -g(3,4);
        dc(6*i-5:6*i, i) = -J(6,:).';   % column gradient
    end

    % --- collision-pair constraints ----------------------------------
    idx = N;
    for k = 1:N
        pk  = P(:,k);
        row = (6*(k-1)+1):(6*k);

        for icp = 1:ncp
            idx = idx + 1;

            g1  = getTransform(robot, pk, Pairs(icp).body1.bodyName);
            g2  = eye(4);
            out = Pairs(icp).solveContact(g1, g2);

            c(idx) = -out.alpha + 1 + prob.eps_clear;

            % body-1-frame Jacobian of body1
            J1  = geometricJacobian(robot, pk, Pairs(icp).body1.bodyName);
            R1  = g1(1:3,1:3);
            J1  = [R1.' * J1(1:3,:); R1.' * J1(4:6,:)];        % 6x6, body1 frame
            J1  = dinamico_Adjoint(ginv(Pairs(icp).body1.g_JC)) * J1;

            % relative pose / Jacobian
            g12 = Pairs(icp).get_relative(g1, g2);
            R12 = g12(1:3,1:3);
            r12 = g12(1:3,4);
            J12 = -dinamico_Adjoint(ginv(g12)) * J1;            % body-2 frame, 6x6

            alpha   = out.alpha;
            lambda2 = out.lambda2;
            y       = R12.' * (out.x - r12) / alpha;
            y_t     = dinamico_tilde(y);
            [grad2, H2] = Pairs(icp).body2.get_gradH(y);

            dphi2_dq    = grad2.' * [y_t, -1/alpha*eye(3)] * J12;             % 1x6
            d2phi2_dxdq = (1/alpha) * R12 * ...
                          [H2*y_t - dinamico_tilde(grad2), -1/alpha*H2] * J12; % 3x6
            d2phi2_dadq = -(1/alpha) * (grad2 + H2*y).' * ...
                          [y_t, -1/alpha*eye(3)] * J12;                        % 1x6

            dF_dq = [ zeros(1,6);
                      dphi2_dq;
                      lambda2 * d2phi2_dxdq;
                      lambda2 * d2phi2_dadq ];                                 % 6x6

            dz_dq     = -out.J \ dF_dq;     % requires out.J to be 6x6
            dalpha_dq = dz_dq(4, :);         % 1x6
            dc(row, idx) = (-dalpha_dq).';
        end
    end
end