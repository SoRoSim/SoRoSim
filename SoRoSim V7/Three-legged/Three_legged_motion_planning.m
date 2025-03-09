%% Three-legged robot motion planning
clear
close all
load("Datafiles\three_legged_fixed.mat")
% load("Datafiles\Parallel_robot.mat")
load("Datafiles\constrain_surface.mat")
% S1 = S2;
% S1.VLinks(1).ld = {0.9};
% S1.VLinks(3).ld = {0.9};
S1.VLinks(3).E = 5.105e+8;
S1.VLinks(5).E = 5.105e+8;
S1.VLinks(1).E = 5.105e+8;
% S1.g_ini(13,4) = 2*(S1.VLinks(1).L*cosd(60) + S1.VLinks(2).r(0));
S1.VLinks(2).Rho = 3240;


% S1.PlotParameters.ClosePrevious = false;
S1.CVRods{1}(2).UpdateAll;
S1.CVRods{3}(2).UpdateAll;
S1.CVRods{5}(2).UpdateAll;

S1 = S1.Update;
%%
link_index =[1,3,5];
g_des_initial = [0.0000         0   1.0000    0.4
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.6112
                0         0         0    1.0000];
%% Problem 1:
% This involves finding the pose of the two tips such that the end-effector
% achieves the desired pose anywhere in sapce without considering the hole
% constraints.
qu_uq_l0 = zeros(S1.ndof+S1.nact + 6*S1.nCLj,1);
constraints_handle = @(qu_uq_l)cons1_3L(S1, qu_uq_l);
options = optimoptions('fmincon','Display','iter', 'OptimalityTolerance',1e-10,'StepTolerance',1e-15 ,'MaxFunctionEvaluations',2e6,'Algorithm', 'active-set', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
qu_uq_l_final1 = fmincon(@(qu_uq_l)obj1_3L(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options);
tic

%% plot the solution
S1.plotq(qu_uq_l_final1(1:S1.ndof))
plotTransforms(se3(g_des_initial), 'FrameSize',0.05)
radius = 0.05;
[hole_pos, roots] = find_constraint(S1, qu_uq_l_final1(1:S1.ndof), -0.2,[1,3,5]);
plot_constraint(hole_pos, radius, [1,3,5])

%% Problem 2
% This problem requires finding the pose of the two ends of the robot, such
% that the rigid body achieves a certain desired pose while satisfying the
% hole constraints

% qu_uq_l0 = [qu_uq_l_final1; roots];
constraints_handle = @(qu_uq_l)Cons2_3L(S1, qu_uq_l, hole_pos,link_index, radius);

options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-6,'StepTolerance',1e-10 ,'MaxIterations',4e4,'MaxFunctionEvaluations',2e6,'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

lb = zeros(length(qu_uq_l0),1);
ub = zeros(length(qu_uq_l0),1);
lb(1:end- length(link_index)) = -inf;
lb(end-(length(link_index)-1):end) = 0+0.1;
ub(1:end-length(link_index)) = inf;
ub(end-(length(link_index)-1):end) = 1-0.1;


qu_uq_l_final1 = fmincon(@(qu_uq_l)obj1_3L(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
tic
