clear
close all
load("Datafiles\Parallel_robot.mat");
% load("Datafiles\constrain_surface.mat")
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(3).E = 5.105e+7;
S1.PlotParameters.ClosePrevious = false;
S1.CVRods{1}(2).UpdateAll;
S1.CVRods{3}(2).UpdateAll;
S1 = S1.Update;

%%
g_des_final = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.533
                0         0         0    1.0000];

g_des_initial = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.35
                0         0         0    1.0000];

%% Problem 1:
% This involves finding the pose of the two tips such that the end-effector
% achieves the desired pose anywhere in sapce without considering the hole
% constraints.
qu_uq_l0 = zeros(78,1);
constraints_handle = @(qu_uq_l)constraints1(S1, qu_uq_l);
options = optimoptions('fmincon','Display','iter', 'OptimalityTolerance',1e-10,'StepTolerance',1e-15 ,'MaxFunctionEvaluations',2e6,'Algorithm', 'active-set', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
qu_uq_l_final = fmincon(@(qu_uq_l)Objective1(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options);

%% Find a constraining surface that is feasible
figure
S1.plotq(qu_uq_l_final(1:S1.ndof));
[constraint_surface, root1, root2] = find_constraint(S1, qu_uq_l_final(1:S1.ndof), 0);
plot_constraint(constraint_surface);
%% Problem 2
% This problem requires finding the pose of the two ends of the robot, such
% that the rigid body achieves a certain desired pose while satisfying the
% hole constraints

% qu_uq_l0 = zeros(80,1);
qu_uq_l0 = [qu_uq_l_final; root1; root2];
[c, ceq, ~, ~] = Constraints2(S1, qu_uq_l0, constraint_surface)
%%
constraints_handle = @(qu_uq_l)Constraints2(S1, qu_uq_l, constraint_surface);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-10,'StepTolerance',1e-15 ,'MaxFunctionEvaluations',2e6,'Algorithm', 'sqp');%, 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
lb(1:78) = -inf;
lb(79:80) = 0;
ub(1:78) = inf;
ub(79:80) = 1;

qu_uq_l_final1 = fmincon(@(qu_uq_l)Objective2(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
qu_uq_l_final2 = fmincon(@(qu_uq_l)Objective2(S1, qu_uq_l, g_des_final), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);

%% Plot the results
figure
S1.plotq(qu_uq_l_final(1:S1.ndof))
hold on
plot_constraint(constraint_surface)
plotTransforms(se3(g_des_initial),'FrameSize',0.05)



%% Problem 3: Path planning to go from initial desired end effector to final

ndof = S1.ndof;
n_x_t = ndof+18;
total_time = 1;
n_points = 10;
dt = total_time/(n_points-1);
initial_guess = zeros(n_x_t, n_points);

for i = 1:n_x_t
    initial_guess(i,:) = linspace(qu_uq_l_final1(i), qu_uq_l_final2(i), n_points);
end
initial_guess = initial_guess(:);

constraints_handle = @(qu_uq_l)Constraint3(S1, qu_uq_l, n_points, g_des_initial);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-14 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5,'Algorithm','active-set','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

qu_uq_l_con = fmincon(@(qu_uq_l)Objective3(S1, qu_uq_l,g_des_final),initial_guess, [],[],[],[],[],[],constraints_handle,options);
save("Datafiles/zy_rotation_neg15_degs.mat","qu_uq_l_con")
%%
objective_function(S1, qu_uq_l_con,g_des_final)
%% Plot the solution
figure
S1.PlotParameters.ClosePrevious= false;
qs = qu_uq_l_con(:,1:S1.ndof);
gs1 = S1.FwdKinematics(qs(1,:));
plot_constraint(constraint_surface)
for i = 1:5
    S1.plotq(qs(2*i,:))
    hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)