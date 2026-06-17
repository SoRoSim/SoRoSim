function path_q = RRT_planner(Linkage,state_start, state_goal,initial_guess_start, initial_guess_end, params,env)
% env             : environment / obstacles (your struct)
% params          : struct with fields:
%   MaxIter, StepSize, GoalThreshold, GoalSampleRate, EdgeCheckSteps, etc.
%
% Output:
%   path_q : m x N sequence of strain vectors from start to goal
    pos_bounds= env.pos_bounds;
    eul_bounds= env.eul_bounds;

    MaxIter       = params.MaxIter;
    StepSize      = params.StepSize;
    GoalThreshold = params.GoalThreshold;

    % ---- Initialize two trees: T_start and T_goal ----
    T_start(1).state     = state_start;
    T_start(1).parent = 0;

    T_goal(1).state       = state_goal;
    T_goal(1).parent  = 0;

    nStart = 1;
    nGoal  = 1;

    % To remember where the trees met
    idxStartMeet = -1;
    idxGoalMeet  = -1;

    % We’ll alternate which tree is "active"
    growFromStart = true;

    for k = 1:MaxIter

        % --------- 1) Sample random state in strain space ----------
        if rand < params.GoalSampleRate
            % Bias sampling towards the opposite root
            if growFromStart
                state_rand = state_goal;
            else
                state_rand = state_start;
            end
        else
            state_rand = sample_state(pos_bounds, eul_bounds);   % <-- you implement this
        end

        % ----------------------------------------------------------
        % 2) Extend ACTIVE tree (Ta) toward q_rand
        %    and try to CONNECT the OTHER tree (Tb) to the new node
        % ----------------------------------------------------------
        if growFromStart
            [T_start, nStart,  newIdx] = ...
                extendTree(Linkage,T_start, nStart, state_rand, StepSize, env,params);

            if newIdx ~= -1
                % Try to connect T_goal to q_new
                [T_goal, nGoal, connectIdx] = ...
                    connectTrees(Linkage,T_goal, nGoal, T_start(newIdx), StepSize, env, params);

                if connectIdx ~= -1
                    idxStartMeet = newIdx;    % in T_start
                    idxGoalMeet  = connectIdx; % in T_goal
                    break;
                end
            end

        else
            [T_goal, nGoal, newIdx] = ...
                extendTree(Linkage, T_goal, nGoal, state_rand, StepSize, env, params);

            if newIdx ~= -1
                % Try to connect T_start to q_new
                [T_start, nStart, connectIdx] = ...
                    connectTrees(Linkage,T_start, nStart, T_goal(newIdx), StepSize, env, params);

                if connectIdx ~= -1
                    idxStartMeet = connectIdx; % in T_start
                    idxGoalMeet  = newIdx;     % in T_goal
                    break;
                end
            end
        end

        % Alternate which tree we grow from
        growFromStart = ~growFromStart;
    end

    % ---- If failed to connect ----
    if idxStartMeet == -1
        warning('BiRRT:NoPath', 'Failed to connect trees within MaxIter');
        path_q = [];
        return;
    end

    % ---- 3) Extract path from start -> meeting point ----
    pathStart = backtrackPath(T_start, idxStartMeet);  % m x Ns

    % ---- 4) Extract path from goal -> meeting point (in T_goal) ----
    pathGoal = backtrackPath(T_goal, idxGoalMeet);     % m x Ng
    % pathGoal is [q_goal ... q_meet]; we want [q_meet ... q_goal]
    pathGoal = fliplr(pathGoal);

    % ---- 5) Concatenate, but avoid duplicating the meeting state ----
    if isequal(pathStart(:,end), pathGoal(:,1))
        path_q = [pathStart, pathGoal(:,2:end)];
    else
        path_q = [pathStart, pathGoal];
    end
end
