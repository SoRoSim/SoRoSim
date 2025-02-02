clear
close all
load("Datafiles\Parallel_robot.mat");
% load("Datafiles\constrain_surface.mat")
S1.VLinks(1).ld = {0.9};
S1.VLinks(3).ld = {0.9};
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(2).Rho = 2240;
S1.VLinks(3).E = 5.105e+7;
S1.PlotParameters.ClosePrevious = false;
S1.CVRods{1}(2).UpdateAll;
S1.CVRods{3}(2).UpdateAll;
S1 = S1.Update;

%%
g_des_final = [0.0000         0   1.0000    0.1
                0    1.0000         0         0.2
                -1.0000         0    0.0000    -0.4
                0         0         0    1.0000];

g_des_initial = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.35
                0         0         0    1.0000];
<<<<<<< Updated upstream
rotation_angle = 15;
rotation_angle = rotation_angle*(pi/180);
roty = eul2tform([0, rotation_angle,0]);
=======
rotation_angle = 20;
rotation_angle = rotation_angle*(pi/180);
roty = eul2tform([0,0, rotation_angle]);
>>>>>>> Stashed changes
g_des_final = g_des_final*roty;

%% Problem 1:
% This involves finding the pose of the two tips such that the end-effector
% achieves the desired pose anywhere in sapce without considering the hole
% constraints.
qu_uq_l0 = zeros(78,1);
constraints_handle = @(qu_uq_l)constraints1(S1, qu_uq_l);
options = optimoptions('fmincon','Display','iter', 'OptimalityTolerance',1e-10,'StepTolerance',1e-15 ,'MaxFunctionEvaluations',2e6,'Algorithm', 'active-set', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
qu_uq_l_final1 = fmincon(@(qu_uq_l)Objective1(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options);
qu_uq_l_final2 = fmincon(@(qu_uq_l)Objective1(S1, qu_uq_l, g_des_final), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options);


%% Find a constraining surface that is feasible
figure
S1.plotq(qu_uq_l_final1(1:S1.ndof));
S1.plotq(qu_uq_l_final2(1:S1.ndof));
[constraint_surface, root1, root2] = find_constraint(S1, qu_uq_l_final1(1:S1.ndof), 0);
plot_constraint(constraint_surface);
plotTransforms(se3(g_des_initial), 'FrameSize',0.05);
plotTransforms(se3(g_des_final), 'FrameSize',0.05);
%% Problem 2
% This problem requires finding the pose of the two ends of the robot, such
% that the rigid body achieves a certain desired pose while satisfying the
% hole constraints

% qu_uq_l0 = zeros(80,1);
qu_uq_l0 = [qu_uq_l_final1; root1; root2];
constraints_handle = @(qu_uq_l)Constraints2(S1, qu_uq_l, constraint_surface);
<<<<<<< Updated upstream
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-10,'StepTolerance',1e-15 ,'MaxFunctionEvaluations',2e6,'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
=======
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-6,'StepTolerance',1e-8 ,'MaxFunctionEvaluations',2e10,'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
>>>>>>> Stashed changes
lb = zeros(80,1);
ub = zeros(80,1);
lb(1:78) = -inf;
lb(79:80) = 0+0.1;
ub(1:78) = inf;
ub(79:80) = 1-0.1;

tic
qu_uq_l_final1 = fmincon(@(qu_uq_l)Objective2(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
toc
qu_uq_l_final2 = fmincon(@(qu_uq_l)Objective2(S1, qu_uq_l, g_des_final), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);

%%
<<<<<<< Updated upstream
[c, ceq, ~, ~] = Constraints2(S1, qu_uq_l_final2, constraint_surface)
=======
[c, ~] = Objective2(S1, qu_uq_l_final1, g_des_initial)
>>>>>>> Stashed changes

%% Plot the results
S1.PlotParameters.ClosePrevious = false;
S1.plotq(qu_uq_l_final1(1:S1.ndof))
S1.plotq(qu_uq_l_final2(1:S1.ndof))
hold on
plot_constraint(constraint_surface)
plotTransforms(se3(g_des_initial),'FrameSize',0.05)
plotTransforms(se3(g_des_final), 'FrameSize',0.05)


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

<<<<<<< Updated upstream

constraints_handle = @(qu_uq_l)Constraint3(S1, qu_uq_l, n_points, g_des_initial);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-6,'StepTolerance',1e-6 ,'MaxFunctionEvaluations',2e10,'Algorithm','sqp','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
=======
% lambda = 5e3;
constraints_handle = @(qu_uq_l)Constraint3(S1, qu_uq_l, n_points, g_des_initial, g_des_final);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-10,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10,'Algorithm','sqp','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);%EnableFeasibilityMode',true);
>>>>>>> Stashed changes
tic
qu_uq_l_con = fmincon(@(qu_uq_l)Objective4(S1, qu_uq_l, n_points),initial_guess, [],[],[],[],[],[],constraints_handle,options);
toc
save("Datafiles/zy_rotation_neg15_degs.mat","qu_uq_l_con")
%%
Objective4(S1, qu_uq_l_con,g_des_final, 10)
%% Plot the solution
% figure
<<<<<<< Updated upstream
S1.PlotParameters.ClosePrevious= false;
qu_uq_l_con = reshape(qu_uq_l_con, n_x_t, n_points);
qu_uq_l_con = qu_uq_l_con';
qs = qu_uq_l_con(:,1:S1.ndof);
=======

S1.PlotParameters.ClosePrevious= false;
qu_uq_l = reshape(qu_uq_l_con, n_x_t, n_points);
qu_uq_l = qu_uq_l';
qs = qu_uq_l(:,1:S1.ndof);
>>>>>>> Stashed changes
gs1 = S1.FwdKinematics(qs(1,:));
plot_constraint(constraint_surface)
for i = 1:10
    S1.plotq(qs(i,:))
    hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)

%% Make a good initial condition by interpolating the desired pose and solving at each instance
n_points = 10;
n_x_t = 80;
g = interpolate_transformation(g_des_initial, g_des_final, n_points);
qu_uq_l = [qu_uq_l_final1; root1; root2];

constraints_handle = @(qu_uq_l)Constraints2(S1, qu_uq_l, constraint_surface);
options = optimoptions('fmincon','Display','final-detailed','OptimalityTolerance',1e-6,'StepTolerance',1e-8 ,'MaxFunctionEvaluations',2e10,'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
lb = zeros(80,1);
ub = zeros(80,1);
lb(1:78) = -inf;
lb(79:80) = 0+0.1;
ub(1:78) = inf;
ub(79:80) = 1-0.1;
qu_uq_l_final = zeros(n_points, n_x_t);
for i=1:n_points
    qu_uq_l = fmincon(@(qu_uq_l)Objective2(S1, qu_uq_l, g(4*i-3:4*i,:)), qu_uq_l_final(i,:)', [],[],[],[],lb,ub,constraints_handle,options);
    qu_uq_l_final(i,:) = qu_uq_l;
    Objective2(S1,qu_uq_l, g(4*i-3:4*i,:))
end
%%
qs = qu_uq_l_final(:,1:S1.ndof);
plot_constraint(constraint_surface)
for i = 1:5
    S1.plotq(qs(2*i,:))
    hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)

%% Problem 4: path planning through the hole constraint

ndof = S1.ndof;
n_x_t = ndof+20;
total_time = 1;
n_points = 10;
dt = total_time/(n_points-1);
qu_uq_l1 = [qu_uq_l_final1; root1; root2];
<<<<<<< Updated upstream

=======
qu_uq_l2 = [qu_uq_l_final2; root1; root2];
>>>>>>> Stashed changes
initial_guess = zeros(n_x_t, n_points);
for i = 1:n_x_t
    initial_guess(i,:) = linspace(qu_uq_l1(i), qu_uq_l2(i), n_points);
end
initial_guess = initial_guess(:);
lb = ones(n_points,n_x_t)*-inf;
ub = ones(n_points, n_x_t)*inf;
lb(:, 79:80) = 0+0.01;
ub(:, 79:80) = 1-0.01;
lb = lb';
ub = ub';

lb = lb(:);
ub = ub(:);

<<<<<<< Updated upstream
constraints_handle = @(qu_uq_l)Constraints4(S1, qu_uq_l, n_points, g_des_final, constraint_surface);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-6,'StepTolerance',1e-8 ,'MaxFunctionEvaluations',2e10,'Algorithm','active-set','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
tic
qu_uq_l_con = fmincon(@(qu_uq_l)Objective4(S1, qu_uq_l,g_des_initial, n_points),initial_guess, [],[],[],[],lb,ub,constraints_handle,options);
toc
%%
figure
S1.PlotParameters.ClosePrevious= false;
qu_uq_l_con = reshape(qu_uq_l_con, n_x_t, n_points);
qu_uq_l_con = qu_uq_l_con';
qs = qu_uq_l_con(:,1:S1.ndof);
gs1 = S1.FwdKinematics(qs(1,:));
=======
constraints_handle = @(qu_uq_l)Constraints4(S1, qu_uq_l, n_points, g_des_initial, g_des_final, constraint_surface);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-6,'StepTolerance',1e-8 ,'MaxFunctionEvaluations',2e10,'Algorithm','sqp','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
tic
qu_uq_l_con = fmincon(@(qu_uq_l)Objective4(S1, qu_uq_l, n_points),initial_guess, [],[],[],[],lb,ub,constraints_handle,options);
toc
%%
Objective4(S1, qu_uq_l_con,g_des_initial, n_points)
%%
figure
S1.PlotParameters.ClosePrevious= false;
qu_uq_l = reshape(qu_uq_l_con, n_x_t, n_points);
qu_uq_l = qu_uq_l';
%%
qs = qu_uq_l_final(:,1:S1.ndof);
>>>>>>> Stashed changes
plot_constraint(constraint_surface)
for i = 1:5
    S1.plotq(qs(2*i,:))
    hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)