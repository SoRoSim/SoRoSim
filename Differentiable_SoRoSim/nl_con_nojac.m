% ======================================================================
% Nonlinear constraints (NO JACOBIAN):
% c(x) <= 0, ceq = []
% Does NOT return dc/dx, so fmincon uses finite differences.
% ======================================================================
function [c, ceq] = nl_con_nojac(x, prob)

    N = prob.N;
    Linkage = prob.Linkage;
    P = reshape(x, 3, N);

    ceq = [];

    ncp = Linkage.ncp;
    c   = zeros(ncp*N, 1);

    idx = 0;
    for k = 1:N
        g1 = eye(4);
        g1(1:3,4) = P(:,k);

        for icp = 1:ncp
            idx = idx + 1;

            g2 = eye(4);

            % -------- solve contact --------
            out = Linkage.Pairs(icp).solveContact(g1, g2);

            % constraint: c = -alpha + 1 + eps
            c(idx) = -out.alpha + 1 + prob.eps_clear;
        end
    end
end
