clear
close all
load("Datafiles\Parallel_robot.mat");
% load("Datafiles\constrain_surface.mat")
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(3).E = 5.105e+7;
S1.CVRods{1}(2).UpdateAll;
S1.CVRods{3}(2).UpdateAll;
S1 = S1.Update;
%%
g_des_final = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.433
                0         0         0    1.0000];

g_des_initial = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.35
                0         0         0    1.0000];
% theta = 15*pi/180;
% rotation_y = eul2tform([theta theta 0], 'ZYX');
% T_translate = eye(4);
% T_translate(1:3,4) = [(constraint_surface.hole_1 + constraint_surface.hole_2)/2 constraint_surface.height];
% Transform = T_translate*rotation_y*ginv(T_translate);
% g_des_final = Transform*g_des_final;
%%
qu_uq_l0 = zeros(80,1);
% qu_uq_l0 = qu_uq_l;


constraints_handle = @(qu_uq_l)constraints_instance(S1, qu_uq_l, constraint_surface);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-10,'StepTolerance',1e-15 ,'MaxFunctionEvaluations',2e6,'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
lb(1:78) = -inf;
lb(79:80) = 0;
ub(1:78) = inf;
ub(79:80) = 1;

qu_uq_l_final = fmincon(@(qu_uq_l)objective_function_instance(S1, qu_uq_l, g_des_final), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
% qu_uq_l_initial = fmincon(@(qu_uq_l)objective_function_instance(S1, qu_uq_l, g_des_initial), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);

%% checking the constraints


%% 
ndof = S1.ndof;
n_x_t = ndof+20;
total_time = 1;
n_points = 10;
dt = total_time/(n_points-1);
initial_guess = [];
for i = 1:n_x_t
    initial_guess = [initial_guess linspace(qu_uq_l_initial(i), qu_uq_l_final(i),n_points)'];
end

lb(1:10,1:78) = -inf;
lb(1:10, 79:80) = 0;
ub(1:10, 1:78) = inf;
ub(1:10, 79:80) = 1;

constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l, n_points, constraint_surface, g_des_initial);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-14 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5,'Algorithm','sqp');%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

qu_uq_l_con = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l,g_des_final),initial_guess, [],[],[],[],lb,ub,constraints_handle,options);
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