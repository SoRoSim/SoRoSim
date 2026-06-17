function [T, nNodes, newIdx] = extendTree(Linkage,T, nNodes, state_target, stepSize, env, params)
    % Find nearest node
    idxNear = findClosestNode(T,state_target);
    state_near = T(idxNear).state;

    % Steer
    [state_new, q_a, x] = steer(Linkage, T(idxNear), state_target, stepSize);  % your steer in strain space
    
    % if ~success
    %     newIdx = -1;
    %     return;
    % end

    % Check the edge feasibility (state_near -> state_new)
    if ~edgeFeasible(Linkage,T(idxNear), q_a, x, env, params)
        % success=
        newIdx = -1;
        return;
    end

    % Add node
   
    nNodes = nNodes + 1;
    T(nNodes).state  = state_new;
    T(nNodes).parent = idxNear;
    T(nNodes).q_a = q_a;
    T(nNodes).x = x;
    Linkage.plotq(strain_parameters(Linkage,T(nNodes)));
    newIdx = nNodes;
end