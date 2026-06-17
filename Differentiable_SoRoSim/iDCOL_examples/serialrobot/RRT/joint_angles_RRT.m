%% RRT using the joint angles
%% RRT comparsion
clear
load("Linkages\Experimental_linkage.mat")
% Linkage = full_robot;
baxter_tree = importrobot("urdf\new_baxter.urdf");
% Linkage.CVRods{11}(2).Phi_odr = [1;1;1;1;1;1]*0;
% Linkage.CVRods{9}(2).Phi_odr = [1;1;1;1;1;1]*0;
% Linkage.CVRods{9}(2).UpdateAll;
% Linkage.CVRods{11}(2).UpdateAll;
% Linkage = Linkage.Update;

left_hand = 'left_hand';
right_hand = 'right_hand';
% ik = inverseKinematics('RigidBodyTree',baxter_tree);
joint_left = dictionary( 'left_e0',  -0.034547, ...
                        'left_e1', 2.0042, ...
                        'left_s0', -0.58664, ...
                        'left_s1', -1.137,...
                        'left_w0', 0.21857,...
                        'left_w1', -0.87636, ...
                        'left_w2', -0.12647, ...
                        'right_e0',  0.50658, ...
                        'right_e1', 1.984, ...
                        'right_s0', 0.22123, ...
                        'right_s1', -1.0741, ...
                        'right_w0', -0.14492, ...
                        'right_w1', -0.83544, ...
                        'right_w2', -0.13709);

config_Sol = dict2config1(joint_left,baxter_tree);

initial_config = config_Sol;

[q_left, q_right] = config2vector(initial_config);
% action = [[-0.7; -0.7;0.2;0;0;0.3];[0.7; -0.5;0.2;0.3;0.6;0];  [0.8; -0.4;0.2;0.3;-0.6;0]]
action =[q_left+[-0*pi/180;0;0;0;0;0;0*pi/180];q_right+[0*pi/180;0;0;0;0;0;-0*pi/180]];

%% initial_State and final state
load("Experiments\Sorosim_trajectories\physical_constraints\constrained_3.mat")
q_a_init = [qu_uq_lfull(1,1:7)'; qu_uq_lfull(1,Linkage.ndof-6:Linkage.ndof)'];
initial_guess_start = [qu_uq_lfull(1,8:Linkage.ndof-7)';qu_uq_lfull(1,Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; qu_uq_lfull(1, Linkage.ndof+Linkage.nact+1:end-2)'];

q_a_fin = [qu_uq_lfull(10,1:7)'; qu_uq_lfull(10,Linkage.ndof-6:Linkage.ndof)'];
initial_guess_end = [qu_uq_lfull(10,8:Linkage.ndof-7)';qu_uq_lfull(10,Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; qu_uq_lfull(10, Linkage.ndof+Linkage.nact+1:end-2)'];
%% check if the goal state and the end state are feasible
%% Check inverse kinematics code

%% 
RRT_params.MaxIter = 1e5;
RRT_params.MaxiterConnect = 40;
RRT_params.StepSize= 0.04;
RRT_params.GoalThreshold = 0.5e-2;
RRT_params.GoalSampleRate = 0.05;
RRT_params.EdgeCheckSteps = 10;
RRT_params.Plot = false;

env.Radius= 0.06;
env.hole1 = [0.8496 0.1489 0.07];
env.hole2 = [0.8676 -0.1506 0.0790];
env.joint_angle_bounds = [[-1.70167993878; -2.147; -3.05417993878; -0.05; -3.059; -1.57079632679; -3.059] [1.70167993878; 1.047; 3.05417993878; 2.618; 3.059; 2.094; 3.059]];
%%
% plotq_urdf(Linkage, qu_uq_lfull(10,1:Linkage.ndof), 0.05, baxter_tree, 1, [8,19])
% plot_constraint([env.hole1;env.hole2], env.Radius, [8,19]);
init_feasible = isFeasible(Linkage, qu_uq_lfull(10, 1:Linkage.ndof)',env)
init_feasible = isFeasible(Linkage, qu_uq_lfull(1, 1:Linkage.ndof)',env)
%%
tic
[RRT_plan, RRT_path_x] = RRT_planner(Linkage, q_a_init,q_a_fin,initial_guess_start, initial_guess_end,RRT_params, env);
toc
%%
% plot the planned trajectory
% load("RRT_saved_runs\rrt_out_constrained_1_run_001.mat");
N = size(RRT_path_x, 2);   % number of columns (points along path)
idx = round(linspace(1, N, 10));
for i = idx
    j = i/size(RRT_path_x,2);
    q= [RRT_plan(1:7,i); RRT_path_x(1:Linkage.ndof-14,i); RRT_plan(8:14,i)];
    plot_free_manipulation(Linkage,q, (j)^1.5);
end
plot_constraint(hole_position,radius, [1,4])
%% check objective function for RRT
N = size(RRT_path_x, 2);   % number of columns (points along path)
idx = round(linspace(1, N, 10));
matrix_form = [RRT_plan(1:7,idx); RRT_path_x(1:Linkage.ndof-14,idx); RRT_plan(8:14,idx); RRT_path_x(Linkage.ndof+1-Linkage.nact:end,idx)]';
vec= reshape(matrix_form(2:10,:)',[],1);
Objective4_baxter_multistep(Linkage, vec, 10, matrix_form(1,:)')
%% Crash-safe benchmark of RRT_planner

resultsFile = 'RRT_timing_results3.txt';

% Where to store per-run outputs
saveDir = 'RRT_saved_runs';
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

% Fixed RRT parameters
RRT_params.MaxIter         = 1e5;
RRT_params.MaxiterConnect  = 40;
RRT_params.StepSize        = 0.01;
RRT_params.GoalThreshold   = 0.5e-2;
RRT_params.GoalSampleRate  = 0.05;
RRT_params.EdgeCheckSteps  = 10;
RRT_params.Plot            = false;

% Environment
env.Radius = 0.06;
env.hole1  = [0.8496  0.1489  0.07];
env.hole2  = [0.8676 -0.1506  0.0790];
env.joint_angle_bounds = [[-1.70167993878; -2.147; -3.05417993878; -0.05; -3.059; -1.57079632679; -3.059] ...
                          [ 1.70167993878;  1.047;  3.05417993878;  2.618;  3.059;  2.094;          3.059]];
N_runs = 20;

for file_idx = 1:4

    matFile = sprintf( ...
        'Experiments\\Sorosim_trajectories\\physical_constraints\\constrained_%d.mat', ...
        file_idx);

    fprintf('Loading %s\n', matFile);
    load(matFile, 'qu_uq_lfull', 'Linkage');

    % build start / goal states
    q_a_init = [ ...
        qu_uq_lfull(1, 1:7)'; ...
        qu_uq_lfull(1, Linkage.ndof-6:Linkage.ndof)' ];

    initial_guess_start = [ ...
        qu_uq_lfull(1, 8:Linkage.ndof-7)'; ...
        qu_uq_lfull(1, Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; ...
        qu_uq_lfull(1, Linkage.ndof+Linkage.nact+1:end-2)' ];

    q_a_fin = [ ...
        qu_uq_lfull(10, 1:7)'; ...
        qu_uq_lfull(10, Linkage.ndof-6:Linkage.ndof)' ];

    initial_guess_end = [ ...
        qu_uq_lfull(10, 8:Linkage.ndof-7)'; ...
        qu_uq_lfull(10, Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; ...
        qu_uq_lfull(10, Linkage.ndof+Linkage.nact+1:end-2)' ];

    % ---- open results file in APPEND mode ----
    fid = fopen(resultsFile, 'a');
    if fid == -1
        error('Could not open %s for writing.', resultsFile);
    end

    % write header immediately
    fprintf(fid, '=============================================\n');
    fprintf(fid, 'File: %s\n', matFile);
    fprintf(fid, 'Start time: %s\n', datestr(now));
    fprintf(fid, 'Run\tTime (s)\tSavedMat\n');
    drawnow

    times = zeros(N_runs,1);

    for k = 1:N_runs
        t_start = tic;

        [RRT_plan, RRT_path_x] = RRT_planner( ...
            Linkage, ...
            q_a_init, q_a_fin, ...
            initial_guess_start, initial_guess_end, ...
            RRT_params, env);

        times(k) = toc(t_start);

        % ---- save this run immediately (crash-safe) ----
        saveName = sprintf('rrt_out_constrained_%d_run_%03d.mat', file_idx, k);
        savePath = fullfile(saveDir, saveName);

        % Save everything you'd want to inspect later
        run_info.file_idx            = file_idx;
        run_info.run_idx             = k;
        run_info.source_matFile      = matFile;
        run_info.timestamp           = datestr(now);
        run_info.time_seconds        = times(k);

        run_info.RRT_params          = RRT_params;
        run_info.env                 = env;

        run_info.q_a_init            = q_a_init;
        run_info.q_a_fin             = q_a_fin;
        run_info.initial_guess_start = initial_guess_start;
        run_info.initial_guess_end   = initial_guess_end;

        save(savePath, 'RRT_plan', 'RRT_path_x', 'run_info', '-v7.3');

        % write this run immediately
        fprintf(fid, '%3d\t%.6f\t%s\n', k, times(k), savePath);
        drawnow

        fprintf('File %d | Run %3d/%3d | Time = %.3f s | Saved: %s\n', ...
            file_idx, k, N_runs, times(k), saveName);
    end

    % write summary after finishing this file
    fprintf(fid, '--- Summary ---\n');
    fprintf(fid, 'Mean time (s): %.6f\n', mean(times));
    fprintf(fid, 'Std  time (s): %.6f\n', std(times));
    fprintf(fid, 'Min  time (s): %.6f\n', min(times));
    fprintf(fid, 'Max  time (s): %.6f\n', max(times));
    fprintf(fid, 'End time: %s\n\n', datestr(now));

    fclose(fid);
end

fprintf('Crash-safe timing complete.\n');
fprintf('Timing log: %s\n', resultsFile);
fprintf('Saved runs folder: %s\n', saveDir);

% function [path_q, path_x] = RRT_planner(Linkage,state_start, state_goal,initial_guess_start, initial_guess_end, params,env, plot)
% % RRT_planner (modified to log infeasible candidate counts and nearest distances)
%     joint_bounds= env.joint_angle_bounds;
% 
%     MaxIter       = params.MaxIter;
%     StepSize      = params.StepSize;
%     GoalThreshold = params.GoalThreshold;
% 
%     % ---- Initialize two trees: T_start and T_goal ----
%     T_start(1).q_a     = state_start;
%     [q, u, l] = Linkage.statics(initial_guess_start, T_start(1).q_a,'plot',false);
%     T_start(1).x = [q(8:end-7);u;l];
%     T_start(1).parent = 0;
% 
%     T_goal(1).q_a      = state_goal;
%     [q, u, l] = Linkage.statics(initial_guess_end, T_goal(1).q_a,'plot',false);
%     T_goal(1).x = [q(8:end-7);u;l];
%     T_goal(1).parent  = 0;
% 
%     nStart = 1;
%     nGoal  = 1;
% 
%     % To remember where the trees met
%     idxStartMeet = -1;
%     idxGoalMeet  = -1;
% 
%     % We'll alternate which tree is "active"
%     growFromStart = true;
% 
%     % global counters (optional if you want totals)
%     totalInfeasibleExtend = 0;
%     totalInfeasibleConnect = 0;
% 
%     for k = 1:MaxIter
% 
%         % --------- 1) Sample random state in strain space ----------
%         if rand < params.GoalSampleRate
%             % Bias sampling towards the opposite root
%             if growFromStart
%                 state_rand = state_goal;
%             else
%                 state_rand = state_start;
%             end
%         else
%             state_rand = sample_state(joint_bounds);
%         end
% 
%         % compute distance from sampled state to nearest node in active tree
%         if growFromStart
%             if isempty(T_start)
%                 distToNearest = NaN;
%             else
%                 idxNear_tmp = findClosestNode(T_start, state_rand);
%                 distToNearest = dist(T_start(idxNear_tmp).q_a, state_rand);
%             end
%         else
%             if isempty(T_goal)
%                 distToNearest = NaN;
%             else
%                 idxNear_tmp = findClosestNode(T_goal, state_rand);
%                 distToNearest = dist(T_goal(idxNear_tmp).q_a, state_rand);
%             end
%         end
% 
%         % initialize per-iteration counters
%         infeasibleThisIter = 0;
% 
%         % ----------------------------------------------------------
%         % 2) Extend ACTIVE tree (Ta) toward q_rand
%         %    and try to CONNECT the OTHER tree (Tb) to the new node
%         % ----------------------------------------------------------
%         if growFromStart
%             [T_start, nStart,  newIdx, nInfeasExt] = ...
%                 extendTree(Linkage,T_start, nStart, state_rand, StepSize, env,params);
% 
%             infeasibleThisIter = infeasibleThisIter + nInfeasExt;
%             totalInfeasibleExtend = totalInfeasibleExtend + nInfeasExt;
% 
%             if newIdx ~= -1
%                 % Try to connect T_goal to q_new
%                 [T_goal, nGoal, connectIdx, nInfeasConn] = ...
%                     connectTrees(Linkage,T_goal, nGoal, T_start(newIdx), StepSize, env, params);
% 
%                 infeasibleThisIter = infeasibleThisIter + nInfeasConn;
%                 totalInfeasibleConnect = totalInfeasibleConnect + nInfeasConn;
% 
%                 if connectIdx ~= -1
%                     idxStartMeet = newIdx;    % in T_start
%                     idxGoalMeet  = connectIdx; % in T_goal
%                     % print final connecting info and break
%                     % fprintf('[%4d] dist=%.6f | infeasible(ext)=%d infeasible(conn)=%d --> CONNECTED\n', ...
%                     %     k, distToNearest, nInfeasExt, nInfeasConn);
%                     break;
%                 end
%             end
% 
%         else
%             [T_goal, nGoal, newIdx, nInfeasExt] = ...
%                 extendTree(Linkage, T_goal, nGoal, state_rand, StepSize, env, params);
% 
%             infeasibleThisIter = infeasibleThisIter + nInfeasExt;
%             totalInfeasibleExtend = totalInfeasibleExtend + nInfeasExt;
% 
%             if newIdx ~= -1
%                 % Try to connect T_start to q_new
%                 [T_start, nStart, connectIdx, nInfeasConn] = ...
%                     connectTrees(Linkage,T_start, nStart, T_goal(newIdx), StepSize, env, params);
% 
%                 infeasibleThisIter = infeasibleThisIter + nInfeasConn;
%                 totalInfeasibleConnect = totalInfeasibleConnect + nInfeasConn;
% 
%                 if connectIdx ~= -1
%                     idxStartMeet = connectIdx; % in T_start
%                     idxGoalMeet  = newIdx;     % in T_goal
%                     % fprintf('[%4d] dist=%.6f | infeasible(ext)=%d infeasible(conn)=%d --> CONNECTED\n', ...
%                     %     k, distToNearest, nInfeasExt, nInfeasConn);
%                     break;
%                 end
%             end
%         end
% 
%         % print iteration summary (every iteration)
%         % fprintf('[%4d] dist=%.6f | infeasible_this_iter=%d | totalInfeasExt=%d totalInfeasConn=%d\n', ...
%         %     k, distToNearest, infeasibleThisIter, totalInfeasibleExtend, totalInfeasibleConnect);
% 
%         % Alternate which tree we grow from
%         growFromStart = ~growFromStart;
%     end
% 
%     % ---- If failed to connect ----
%     if idxStartMeet == -1
%         warning('BiRRT:NoPath', 'Failed to connect trees within MaxIter');
%         path_q = [];
%         path_x = [];
%         return;
%     end
% 
%     % ---- 3) Extract path from start -> meeting point ----
%     [pathStart_qa, pathStart_x] = backtrackPath(T_start, idxStartMeet);  % m x Ns
% 
%     % ---- 4) Extract path from goal -> meeting point (in T_goal) ----
%     [pathGoal_qa, pathGoal_x] = backtrackPath(T_goal, idxGoalMeet);     % m x Ng
%     % pathGoal is [q_goal ... q_meet]; we want [q_meet ... q_goal]
%     pathGoal_qa = fliplr(pathGoal_qa);
%     pathGoal_x = fliplr(pathGoal_x);
% 
%     % ---- 5) Concatenate, but avoid duplicating the meeting state ----
%     if isequal(pathStart_qa(:,end), pathGoal_qa(:,1))
%         path_q = [pathStart_qa, pathGoal_qa(:,2:end)];
%         path_x = [pathStart_x, pathGoal_x(:,2:end)];
%     else
%         path_q = [pathStart_qa, pathGoal_qa];
%         path_x = [pathStart_x, pathGoal_x];
%     end
% end
% 
% %% ----------------- modified extendTree -----------------
% function [T, nNodes, newIdx, nInfeasible] = extendTree(Linkage,T, nNodes, state_target, stepSize, env, params)
%     % Find nearest node
%     nInfeasible = 0;
%     idxNear = findClosestNode(T,state_target);
% 
%     % Steer
%     [q_a, x,ok] = steer(Linkage, T(idxNear), state_target, stepSize);  % your steer in strain space
% 
%     if ~ok
%         newIdx = -1;
%         nInfeasible = nInfeasible + 1;   % steer failed
%         return;
%     end
% 
%     % Check the edge feasibility (state_near -> state_new)
%     if ~edgeFeasible(Linkage,T(idxNear), q_a, x, env, params)
%         newIdx = -1;
%         nInfeasible = nInfeasible + 1;   % edge check failed
%         return;
%     end
% 
%     % Add node
%     nNodes = nNodes + 1;
%     T(nNodes).parent = idxNear;
%     T(nNodes).q_a = q_a;
%     T(nNodes).x = x;
%     if isfield(params,'Plot') && params.Plot 
%         Linkage.plotq(strain_parameters(Linkage,T(nNodes)));
%     end
%     newIdx = nNodes;
% end
% 
% %% ----------------- modified connectTrees -----------------
% function [T, nNodes, meetIdx, nInfeasible] = connectTrees(Linkage,T, nNodes, new_node, stepSize, env, params)
%     meetIdx = -1;
%     iterConnect = 0;
%     nInfeasible = 0;
%     while true
%         % Nearest node to the target
%         iterConnect = iterConnect+1;
%         if iterConnect> params.MaxiterConnect
%             return;
%         end
%         idxNear = findClosestNode(T,new_node.q_a);
%         q_near = T(idxNear).q_a;
%         q_target = new_node.q_a;
% 
%         % If already close enough, we consider it connected
%         if dist(q_near, q_target) < params.GoalThreshold
%             meetIdx = idxNear;
%             return;
%         end
% 
%         % Take one step
%         [q_a, x,ok] = steer(Linkage, T(idxNear), new_node.q_a, stepSize);
%         if ~ok
%             nInfeasible = nInfeasible + 1;
%             return;
%         end
%         % Check feasibility along the edge
%         if ~edgeFeasible(Linkage,T(idxNear), q_a, x, env, params)
%             nInfeasible = nInfeasible + 1;
%             return;  % cannot advance further toward target
%         end
% 
%         % Add new node to the tree
%         nNodes = nNodes + 1;
%         T(nNodes).parent = idxNear;
%         T(nNodes).q_a = q_a;
%         T(nNodes).x = x;
%         if isfield(params,'Plot') && params.Plot
%             q_new = strain_parameters(Linkage, T(nNodes));
%             Linkage.plotq(q_new);
%             axis tight
%         end
%         % If we got close enough to the target, we're done
%         if dist(q_a, q_target) < params.GoalThreshold
%             meetIdx = nNodes;
%             return;
%         end
%     end
% end
% 
% %% ----------------- rest of helper functions remain unchanged -----------------
% % (steer, findClosestNode, connectTrees, dist, backtrackPath, edgeFeasible, etc.)
% % make sure their signatures match the calls above (steer returns [state_new,x,ok])
% 
% 
% 
% function q_rand = sample_state(joint_bounds)
% % randSE3  
%     q_rand= zeros(14,1);
%     for i=1:7
%         % --- Random position ---
%         q_rand(i) = joint_bounds(i,1) + (joint_bounds(i,2) - joint_bounds(i,1)) * rand;
%         q_rand(i+7) = joint_bounds(i,1) + (joint_bounds(i,2) - joint_bounds(i,1)) * rand;
%     end
% end
% 
% function q = strain_parameters(Linkage,Node)
% %Takes the node in a tree and gives just q
% 
% q = [Node.q_a(1:7); Node.x(1:Linkage.ndof-14); Node.q_a(8:14)];
% end
% 
% 
% function [state_new, x, ok] = steer(Linkage,T_near, state_rand, stepSize)
%     q_a_near = T_near.q_a;
%     dir = state_rand - q_a_near;
%     nrm = norm(dir);
%     if nrm < 1e-5
%         state_new = q_a_near;
%         x = T_near.x;
%         ok = false;
%         return
%     else
%         state_new = q_a_near + stepSize*(dir/nrm);
%     end
%     try
%         [q, u,lambda]  = Linkage.statics(T_near.x,state_new,"plot",false);
%         ok =true;
%     catch
%         % warning('steer:statics_failed: %s', ME.message);
%         state_new = q_a_near;
%         x = T_near.x;
%         ok = false;
%         return;
%     end
%     x = [q(8:end-7);u;lambda];
% end
% 
% function closestNodeIndex = findClosestNode(T, state_target)
%     % Find the closest node in the tree to the target state using logmap distance.
%     % Inputs:
%     %   tree - a structure with a field 'state' containing 4x4 matrices in SE(3)
%     %   state_target - a 4x4 matrix representing the target state in SE(3)
% 
%     numNodes = numel(T);
%     minDistance = inf;
%     closestNodeIndex = -1;
% 
%     for i = 1:numNodes
%         currentState = T(i).q_a; % Extract the current state
% 
%         distance = dist(currentState, state_target);
%         if distance < minDistance
%             minDistance = distance;
%             closestNodeIndex = i;
%         end
%     end
% end
% 
% 
% 
% function d = dist(q1, q2)
% % We cannot  compare the distance between two nodes based on SE(3) distance
% % of the tooltip. the shape of the deformable object or the manipulator arm
% % might be different. So comparison is made based on the strain parameters
%     W = diag(ones(length(q1),1));   % you can weight some strains more
%     d = sqrt((q1 - q2)' * W * (q1 - q2))/length(q1);
% end
% 
% function [path_angle, path_x] = backtrackPath(T, idx)
%     q_dim = numel(T(1).q_a);
%     path_angle  = T(idx).q_a;   % m x 1
%     path_x = T(idx).x;
%     parent = T(idx).parent;
% 
%     while parent ~= 0
%         path_angle = [T(parent).q_a, path_angle];
%         path_x = [T(parent).x,path_x];
%         parent = T(parent).parent;
%     end
% end
% function ok = edgeFeasible(Linkage,T_near, q_a, x, env, params)
% % EDGEFEASIBLE  Checking if the edge is feasible is done through the strain
% % parametrisation.interpolate q and check if any of the steps would be
% % infeasible
% 
% 
%     nSteps = params.EdgeCheckSteps;
%     q1 = [T_near.q_a(1:7); T_near.x(1:Linkage.ndof-14); T_near.q_a(8:14)];
%     q2 = [q_a(1:7); x(1:Linkage.ndof-14); q_a(8:14)];
%     for i = 1:nSteps
%         alpha = i / nSteps;
% 
%         % Interpolate strain vector in strain space
%         q = (1 - alpha) * q1 + alpha * q2;
% 
%         % Check state feasibility (your model-dependent function)
%         if ~isFeasible(Linkage,q, env)
%             ok = false;
%             return;
%         end
%     end
% 
%     ok = true;
% end
% function result = find_xy_at_z_smooth(x, y, z, z_query, opts)
% % find_xy_at_z_smooth
% % Finds all (x,y,z) positions along a rod where z(s) = z_query
% % using smooth spline fits.
% %
% % Inputs:
% %   x, y, z   - Nx1 vectors of Cartesian coordinates along the rod
% %   z_query   - scalar z value (e.g., env.hole(3))
% %   opts      - optional struct:
% %       .method  : 'spline' (default) or 'csaps'
% %       .smoothP : smoothing parameter for csaps (default 1e-6)
% %       .nGrid   : grid resolution for root bracketing (default 2000)
% %       .tol     : root tolerance (default 1e-9)
% %
% % Output struct:
% %   result.xy        - Kx2 array of [x y] intersection points
% %   result.xyz       - Kx3 array of [x y z]
% %   result.s         - Kx1 parameter values where intersections occur
% %   result.ppx,ppy,ppz - spline pp-forms
% %
% 
% if nargin < 5, opts = struct(); end
% if ~isfield(opts,'method'),  opts.method = 'spline'; end
% if ~isfield(opts,'smoothP'), opts.smoothP = 1e-6; end
% if ~isfield(opts,'nGrid'),   opts.nGrid = 2000; end
% if ~isfield(opts,'tol'),     opts.tol = 1e-9; end
% 
% % Ensure column vectors
% x = x(:); y = y(:); z = z(:);
% N = numel(x);
% assert(numel(y)==N && numel(z)==N, 'x,y,z must be same length');
% 
% % Parameterization: arc-length
% ds = sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2);
% s  = [0; cumsum(ds)];
% 
% % Build smooth curves
% switch lower(opts.method)
%     case 'spline'
%         ppx = spline(s, x);
%         ppy = spline(s, y);
%         ppz = spline(s, z);
%     case 'csaps'
%         ppx = csaps(s, x, opts.smoothP);
%         ppy = csaps(s, y, opts.smoothP);
%         ppz = csaps(s, z, opts.smoothP);
%     otherwise
%         error('Unknown method');
% end
% 
% % Root finding for z(s) - z_query = 0
% f = @(ss) ppval(ppz, ss) - z_query;
% 
% s_grid = linspace(s(1), s(end), opts.nGrid);
% f_grid = f(s_grid);
% 
% idx = find(f_grid(1:end-1).*f_grid(2:end) <= 0);
% s_roots = [];
% 
% for k = idx(:)'
%     a = s_grid(k);
%     b = s_grid(k+1);
%     try
%         sr = fzero(f, [a b], optimset('TolX',opts.tol,'Display','off'));
%         if isempty(s_roots) || all(abs(s_roots - sr) > 1e-6)
%             s_roots(end+1,1) = sr; 
%         end
%     catch
%     end
% end
% 
% % Evaluate physical positions
% xq = ppval(ppx, s_roots);
% yq = ppval(ppy, s_roots);
% zq = ppval(ppz, s_roots);
% 
% result.xy  = [xq yq];
% result.xyz = [xq yq zq];
% result.s   = s_roots;
% result.ppx = ppx;
% result.ppy = ppy;
% result.ppz = ppz;
% end
% 
% 
% 
% function feasible = isFeasible(Linkage,q,env)
%     g = Linkage.FwdKinematics(q,Linkage.CP1(1));
%     X = g(5:4:end,4);
%     Y = g(6:4:end,4);
%     Z = g(7:4:end,4);
%     % if min(Z) > env.hole1(3) || max(Z)<env.hole1(3)
%     %     feasible = false; % Set feasible to false if the condition is not met
%     %     return
%     % end
%     % idx = find(Z>= env.hole1(3),1,'last');
% 
%     resL = find_xy_at_z_smooth(X, Y, Z, env.hole1(3));
%     if isempty(resL.xy)
%         feasible = false;
%         return;
%     end
%     d = vecnorm(resL.xy - env.hole1(1:2), 2, 2);
%     [~,kmin] = min(d);
%     pL_cross = resL.xy(kmin,:);
%     % X1 = Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx)+(env.hole1(3) - Z(idx))*(Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx+1) -Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx))/(Z(idx+1)-Z(idx));
%     g = Linkage.FwdKinematics(q,Linkage.CP1(2));
%     X = g(5:4:end,4);
%     Y = g(6:4:end,4);
%     Z = g(7:4:end,4);
%     % if min(Z) > env.hole2(3) || max(Z) < env.hole2(3)
%     %     feasible = false;
%     %     return
%     % end
%     % idx = find(Z>= env.hole2(3),1,'first');
%     % idx =idx-1;
%     resR = find_xy_at_z_smooth(X, Y,Z,env.hole2(3));
%     if isempty(resR.xy)
%         feasible = false;
%         return;
%     end
%     d = vecnorm(resR.xy - env.hole2(1:2), 2, 2);
%     [~,kmin] = min(d);
%     pR_cross = resR.xy(kmin,:);
%     % X2 = Linkage.CVRods{Linkage.CP1(2)}(2).Xs(idx)+(env.hole2(3) - Z(idx))*(Linkage.CVRods{Linkage.CP1(2)}(2).Xs(idx+1) -Linkage.CVRods{Linkage.CP1(2)}(2).Xs(idx))/(Z(idx+1)-Z(idx));
% 
%     if norm(env.hole1(1:2)  - pL_cross)^2 - 1*(env.Radius - 0.0009)^2>0 || norm(env.hole2(1:2) - pR_cross)^2 - 1*(env.Radius - 0.0009)^2>0
%         feasible = false;
%         return
%     end
%     feasible = true;
% end
