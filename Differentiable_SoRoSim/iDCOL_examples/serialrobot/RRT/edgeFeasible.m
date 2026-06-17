function ok = edgeFeasible(Linkage,T_near, q_a, x, env, params)
% EDGEFEASIBLE  Checking if the edge is feasible is done through the strain
% parametrisation.interpolate q and check if any of the steps would be
% infeasible


    nSteps = params.EdgeCheckSteps;
    q1 = [T_near.q_a(1:7); T_near.x(1:Linkage.ndof-14); T_near.q_a(8:14)];
    q2 = [q_a(1:7); x(1:Linkage.ndof-14); q_a(8:14)];
    for i = 1:nSteps
        alpha = i / nSteps;

        % Interpolate strain vector in strain space
        q = (1 - alpha) * q1 + alpha * q2;

        % Check state feasibility (your model-dependent function)
        if ~isFeasible(Linkage,q, env)
            ok = false;
            return;
        end
    end

    ok = true;
end
