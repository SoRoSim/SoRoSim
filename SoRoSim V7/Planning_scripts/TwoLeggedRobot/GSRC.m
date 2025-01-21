%% Object avoidance path planning (GSRC)

clear
close all
load("upside_down_two_legged.mat");
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(3).E = 5.105e+7;
S1.CVTwists{1}(2).UpdateAll;
S1.CVTwists{3}(2).UpdateAll;
S1 = S1.Update;

%% plan a path from initial point to end without an obstacle in the way

g_des_final = [0.0000         0   1.0000    0.3
                0    1.0000         0         0.25
                -1.0000         0    0.0000    -0.433
                0         0         0    1.0000];

g_initial = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.35
                0         0         0    1.0000];
p_obs = [0.47 0.105 -0.162];
r_obs = 0.05;
ndof = S1.ndof;
n_x_t = ndof+20;
total_time = 1;
n_points = 10;
dt = total_time/(n_points-1);
%find q_0 for all time
% qu_uq_l0 = [qu_uq_l ones(10,1)*root1 ones(10,1)*root2];
% qu_uq_l0(:,1:78) = qu_uq_l0(:,1:78);
qu_uq_l0 = zeros(10,80);

lb(1:10,1:78) = -inf;
lb(1:10, 79:80) = 0;
ub(1:10, 1:78) = inf;
ub(1:10, 79:80) = 1;

constraints_handle = @(qu_uq_l)Constraints(S1, qu_uq_l, n_points, g_initial, p_obs, r_obs);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5,'Algorithm','sqp');%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
tic
qu_uq_l_con = fmincon(@(qu_uq_l)Objective_function(S1, qu_uq_l, g_des_final), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
toc
%% Plot the solution
figure
S1.PlotParameters.ClosePrevious= false;
qs = qu_uq_l_con(:,1:S1.ndof);
gs1 = S1.FwdKinematics(qs(1,:));
plotTransforms(se3(g_initial), 'FrameSize',0.05);
for i = 1:10
    S1.plotq(qs(i,:))
    hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)

%% Plot the sphere
[X,Y,Z] = sphere;
X2 = X * r_obs;
Y2 = Y * r_obs;
Z2 = Z * r_obs;
surf(X2+p_obs(1),Y2+p_obs(2),Z2 + p_obs(3))

%% Objective and constraint functions
[c, ceq] = Constraints(S1, qu_uq_l_con, n_points, g_initial, p_obs, r_obs);

function J = Objective_function(S1, qu_uq_l, g_des_final)
    q_T = qu_uq_l(end, 1:S1.ndof);
    g_T = S1.FwdKinematics(q_T, 2);
    g_T = g_T(5:8,1:4);
    dxdt = diff(qu_uq_l(:,1:S1.ndof));
    J = norm(piecewise_logmap(ginv(g_des_final)*g_T)) + sum(sum(dxdt.^2, 2));
end

function [c, ceq] = Constraints(S1, qu_uq_l, n_points, g_platform_0, p_obs, r_obs)
    qul = [qu_uq_l(:,1:S1.ndof)'; qu_uq_l(:,S1.ndof+13:S1.ndof+18)']';
    uq = qu_uq_l(:,S1.ndof+1:S1.ndof+12);
    ceq = [];
    magnifier = 1;
    lsqoptions = optimoptions('lsqlin','Display','off');
    c = [];
    for i =1:n_points
        g = S1.FwdKinematics(qul(i,:));
        position = [g(1:4:end,4) g(2:4:end,4) g(3:4:end,4)];
        distance = sum((position - p_obs).^2,2);
        c = [c; r_obs + 0.002 - distance];
        eq1 = Equilibrium_optim(S1,qul(i,:)',uq(i,:)', magnifier, lsqoptions);
        ceq = [ceq;eq1];
    end
    q0 = qu_uq_l(1,1:S1.ndof);
    g0 = S1.FwdKinematics(q0, 2);
    g0 = g0(5:8,1:4);
    eq_2 = piecewise_logmap(ginv(g_platform_0)*g0);
    ceq = [ceq; eq_2];
end