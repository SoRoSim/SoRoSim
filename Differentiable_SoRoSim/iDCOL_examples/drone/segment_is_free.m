function ok = segment_is_free(pA, pB, Linkage, eps_clear)
% check segment by discretizing a few points (cheap, robust).
% ok = true if all points satisfy c <= 0 for all pairs.

nCheck = 10;  % increase if obstacles are thin
for i = 0:nCheck
    s = i/nCheck;
    p = (1-s)*pA + s*pB;

    g1 = eye(4); g1(1:3,4) = p;
    g2 = eye(4);

    for icp = 1:Linkage.ncp
        out = Linkage.Pairs(icp).solveContact(g1, g2);
        c = -out.alpha + 1 + eps_clear;
        if c > 0
            ok = false;
            return;
        end
    end
end
ok = true;
end
