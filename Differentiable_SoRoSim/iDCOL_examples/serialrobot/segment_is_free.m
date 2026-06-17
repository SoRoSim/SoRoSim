function ok = segment_is_free(pA, pB, robot, eps_clear, pairs)
% check segment by discretizing a few points (cheap, robust).
% ok = true if all points satisfy c <= 0 for all pairs.

nCheck = 10;  % increase if obstacles are thin
for i = 0:nCheck
    s = i/nCheck;
    p = s*pA + (1-s)*pB;

    
    g2 = eye(4);

    for icp = 1:length(pairs)
        g1 = getTransform(robot,p,pairs(icp).body1.bodyName);
        out = pairs(icp).solveContact(g1, g2);
        c = -out.alpha + 1 + eps_clear;
        if ~out.converged 
            disp("Not converged. Collsion failure!!")
        end
        if c > 0
            ok = false;
            return;
        end
    end
end
ok = true;
end
