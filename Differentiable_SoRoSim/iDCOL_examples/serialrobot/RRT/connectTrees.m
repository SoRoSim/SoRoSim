function [T, nNodes, meetIdx] = connectTrees(Linkage,T, nNodes, new_node, stepSize, env, params)
    meetIdx = -1;

    while true
        % Nearest node to the target
        idxNear = findClosestNode(T,new_node.state);
        q_near = strain_parameters(Linkage,T(idxNear));
        q_target = strain_parameters(Linkage,new_node);

        % If already close enough, we consider it connected
        if dist(q_near, q_target) < params.GoalThreshold
            meetIdx = idxNear;
            return;
        end

        % Take one step
       [state_new, q_a, x] = steer(Linkage, T(idxNear), new_node.state, stepSize);

        % Check feasibility along the edge
        if ~edgeFeasible(Linkage,T(idxNear), q_a, x, env, params)
            return;  % cannot advance further toward target
        end

        % Add new node to the tree
        nNodes = nNodes + 1;
        T(nNodes).state  = state_new;
        T(nNodes).parent = idxNear;
        T(nNodes).q_a = q_a;
        T(nNodes).x = x;
        q_new = strain_parameters(Linkage, T(nNodes));
        Linkage.plotq(q_new);
        axis tight
        % If we got close enough to the target, we’re done
        if dist(q_new, q_target) < params.GoalThreshold
            meetIdx = nNodes;
            return;
        end
    end
end