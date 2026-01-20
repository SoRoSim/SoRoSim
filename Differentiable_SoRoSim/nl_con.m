% ======================================================================
% Nonlinear constraints: collision constraints at every knot
% c(x) <= 0, ceq = []
% Provide gradients dc/dx
% ======================================================================
function [c, ceq, dc, dceq] = nl_con(x,prob)
    N = prob.N;
    Linkage = prob.Linkage;
    P = reshape(x,3,N);

    ceq  = [];  dceq = [];

    ncp = Linkage.ncp;
    c  = zeros(ncp*N,1);
    dc = zeros(3*N, ncp*N);

    % translation-only twist Jacobian Tg1: maps pdot -> twist [w; v]
    % Tg1 = [zeros(3,3); eye(3)];   % 6x3

    idx = 0;
    for k = 1:N
        pk = P(:,k);
        g1 = eye(4);
        g1(1:3,4) = pk;

        row = (3*(k-1)+1):(3*k);

        for icp = 1:ncp
            idx = idx + 1;

            g2 = eye(4);

            % -------- solve contact --------
            out = Linkage.Pairs(icp).solveContact(g1, g2);

            % constraint: c = -alpha + 1 + eps
            c(idx) = -out.alpha + 1 + prob.eps_clear;

            % -------- analytic gradient via your IFT pipeline --------
            % relative pose
            g12 = Linkage.Pairs(icp).get_relative(g1, g2);
            R12 = g12(1:3,1:3);
            r12 = g12(1:3,4);

            % J1: body1 twist Jacobian wrt translation
            % J1 = dinamico_Adjoint(ginv(g1)) * Tg1;   % 6x3
            J1 = [zeros(3,3); eye(3)]; 

            alpha   = out.alpha;
            lambda2 = out.lambda2;

            % y = R12'*(x - r12)/alpha
            y = R12'*(out.x - r12)/alpha;
            y_tilde = dinamico_tilde(y);

            [grad2, H2] = Linkage.Pairs(icp).body2.get_gradH(y);

            % relative Jacobian of 2 wrt 1 in frame of 2 (J2 = 0 for fixed env)
            J12 = -dinamico_Adjoint(ginv(g12)) * J1;  % 6x3

            dphi2_dq    = grad2' * [y_tilde, -1/alpha*eye(3)] * J12;  % 1x3

            d2phi2_dxdq = (1/alpha) * R12 * ...
                [H2*y_tilde - dinamico_tilde(grad2), -1/alpha*H2] * J12; % 3x3

            d2phi2_dadq = -(1/alpha) * (grad2 + H2*y)' * ...
                [y_tilde, -1/alpha*eye(3)] * J12; % 1x3

            % IMPORTANT: dF_dq must match out.J size (typically 6x6)
            % Here q has size 3 (translation only)
            dF_dq = [zeros(1,3);
                     dphi2_dq;
                     lambda2 * d2phi2_dxdq;
                     lambda2 * d2phi2_dadq]; % 6x3

            dz_dq = - out.J \ dF_dq;     % 6x3

            % alpha index: you used row 4 in your code
            dalpha_dq = dz_dq(4,:);      % 1x3

            % c = -alpha + const => dc/dq = -dalpha/dq
            dc(row, idx) = (-dalpha_dq).';
        end
    end
end
