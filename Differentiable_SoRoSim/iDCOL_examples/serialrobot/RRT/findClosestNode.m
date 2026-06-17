function closestNodeIndex = findClosestNode(T, state_target)
    % Find the closest node in the tree to the target state using logmap distance.
    % Inputs:
    %   tree - a structure with a field 'state' containing 4x4 matrices in SE(3)
    %   state_target - a 4x4 matrix representing the target state in SE(3)
    
    numNodes = numel(T);
    minDistance = inf;
    closestNodeIndex = -1;

    for i = 1:numNodes
        currentState = T(i).state; % Extract the current state
        
        % Compute the logmap distance
        distance = norm(piecewise_logmap(ginv(currentState(1:4,:))*state_target(1:4,:)))+norm(piecewise_logmap(ginv(currentState(5:8,:))*state_target(5:8,:)));
        
        if distance < minDistance
            minDistance = distance;
            closestNodeIndex = i;
        end
    end
end

