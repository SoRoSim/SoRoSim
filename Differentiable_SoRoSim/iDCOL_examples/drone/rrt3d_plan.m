function [pathPts, ok] = rrt3d_plan(p0, pT, lbP, ubP, Linkage, eps_clear, prm)
% Returns pathPts as 3xM polyline from p0 to pT (collision-free), ok flag.

maxIter = prm.maxIter;
step    = prm.step;
goalTol = prm.goalTol;
pGoal   = prm.pGoal;

% Tree storage
V = p0(:);             % 3 x nNodes
parent = 0;            % 1 x nNodes (parent indices)
ok = false;

for it = 1:maxIter
    % sample
    if rand < pGoal
        ps = pT;
    else
        ps = lbP + (ubP-lbP).*rand(3,1);
    end

    % nearest neighbor
    [~, idx] = min(sum((V - ps).^2,1));
    pNear = V(:,idx);

    % steer
    d = ps - pNear;
    dn = norm(d);
    if dn < 1e-12
        continue;
    end
    pNew = pNear + (step/dn)*d;

    % clamp
    pNew = min(max(pNew, lbP), ubP);

    % collision check along segment
    if ~segment_is_free(pNear, pNew, Linkage, eps_clear)
        continue;
    end

    % add node
    V(:,end+1) = pNew;
    parent(end+1) = idx;

    % try connect to goal
    if norm(pNew - pT) <= goalTol
        if segment_is_free(pNew, pT, Linkage, eps_clear)
            V(:,end+1) = pT;
            parent(end+1) = size(V,2)-1;
            ok = true;
            break;
        end
    end
end

if ~ok
    pathPts = [];
    return;
end

% backtrack
k = size(V,2);
pts = V(:,k);
while parent(k) ~= 0
    k = parent(k);
    pts = [V(:,k), pts]; %#ok<AGROW>
end
pathPts = pts;
end
