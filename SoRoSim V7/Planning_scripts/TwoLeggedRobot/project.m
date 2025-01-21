clear
close all
load("two_legged_platform.mat");
%%
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(3).E = 5.105e+7;
S1.CVTwists{1}(2).UpdateAll;
S1.CVTwists{3}(2).UpdateAll;
S1 = S1.Update;
%%
qu_uq_l0 = zeros(S1.ndof+18,1);
rot_y = eul2tform([10*pi/180 90*pi/180 0]);
g_desired = [0.0000         0   -1.0000    0.4
         0    1.0000         0         0
    1.0000         0    0.0000    0.65
         0         0         0    1.0000];
g_desired = g_desired*rot_y;


constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-20 ,'MaxFunctionEvaluations',2e10,'Algorithm', 'sqp');%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

qu_uq_l = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l, g_desired), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options)
%%
objective_function(S1,qu_uq_l,g_desired)
%%

S1.plotq(qu_uq_l(1:S1.ndof))
q = qu_uq_l(1:S1.ndof);
g = S1.FwdKinematics(q);
g_platform = g(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
g_base = g(1:4,1:4);
g_tip = g(69:72,1:4);%% generalize later
hold on
plotTransforms(se3(g_desired),'FrameSize',0.05)
plotTransforms(se3(g_platform), 'FrameSize',0.05)
plotTransforms(se3(g_tip), 'FrameSize',0.05);
plotTransforms(se3(g_base),'FrameSize',0.05);
%% incremental initial guess update
rot_y = eul2tform([10*pi/180 90*pi/180 0]);
g_desired = [0.0000         0   -1.0000    0.4
         0    1.0000         0         0
    1.0000         0    0.0000    0.65
         0         0         0    1.0000];
g_desired = g_desired*rot_y;
gd = interpolate_transformation(g_desired, 10);
qs = zeros(length(gd)/4,S1.ndof);

qu_uq_l = qu_uq_l0;
xs=[];
for i =1:length(gd)/4
    constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l);
    options = optimoptions('fmincon','Display','final-detailed','OptimalityTolerance',1e-20,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20); 
    qu_uq_l = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l, gd(4*i-3:4*i,1:4)), qu_uq_l, [],[],[],[],[],[],constraints_handle,options);
    q = qu_uq_l(1:S1.ndof);
    qs(i,:) = q;
    xs=[xs;qu_uq_l];
end
clearvars -except qs S1 trajectory t gd
save(strcat(trajectory,".mat"))
%% Quasi static trajectory tracking (make trajectory and plot it)
g_0 = [0.0000         0   -1.0000    0.3
         0    1.0000         0         0
    1.0000         0    0.0000    0.6
         0         0         0    1.0000];
trajectory = "z_spiral with rotation_x";
[t,gd] = generate_trajectory(trajectory,5,g_0);
qs = zeros(length(t),S1.ndof);
qu_uq_l = zeros(S1.ndof+18,1);

for i =1:length(t)
    constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l);
    options = optimoptions('fmincon','Display','final-detailed','OptimalityTolerance',1e-20,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
    qu_uq_l = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l, gd(4*i-3:4*i,1:4)), qu_uq_l, [],[],[],[],[],[],constraints_handle,options);
    q = qu_uq_l(1:S1.ndof);
    qs(i,:) = q;
end
clearvars -except qs S1 trajectory t gd
save(strcat(trajectory,".mat"))

%% object avoidance

g_desired = [0.0000         0   -1.0000    0.3
         0    1.0000         0         0
    1.0000         0    0.0000    1.2
         0         0         0    1.0000];
p_obs = [0.45,0, 0.9];
r_obs = 0.05;


constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l, p_obs,r_obs);
options = optimoptions('fmincon','Display','final','OptimalityTolerance',1e-20,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

qu_uq_l = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l, g_desired), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options);
q = qu_uq_l(1:S1.ndof);
%%
final_val = objective_function(S1, qu_uq_l,g_desired)

S1.plotq(qu_uq_l(1:S1.ndof))
q = qu_uq_l(1:S1.ndof);
g = S1.FwdKinematics(q);
g_platform = g(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
g_base = g(1:4,1:4);
g_tip = g(69:72,1:4);%% generalize later
hold on
plotTransforms(se3(g_desired),'FrameSize',0.05)
plotTransforms(se3(g_platform), 'FrameSize',0.05)
plotTransforms(se3(g_tip), 'FrameSize',0.05);
plotTransforms(se3(g_base),'FrameSize',0.05);
[X,Y,Z] = sphere;
X2 = X * r_obs;
Y2 = Y * r_obs;
Z2 = Z * r_obs;
surf(X2+p_obs(1),Y2+p_obs(2),Z2 + p_obs(3))
%%

   
function E = objective_function(S1, qu_uq_l, g_desired)
    q = qu_uq_l(1:S1.ndof);
    gs = S1.FwdKinematics(q);
    g_platform = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    E = norm(piecewise_logmap(ginv(g_desired)*g_platform));
end

function [c, ceq] = constraints(S1, qu_uq_l)
    qul = [qu_uq_l(1:S1.ndof)'; qu_uq_l(S1.ndof+13:S1.ndof+18)'];
    uq = qu_uq_l(S1.ndof+1:S1.ndof+12);
%     q = qu_uq_l(1:S1.ndof);
%     gs = S1.FwdKinematics(q);
%     for i =1:length(gs)/4
%         distances(i) = norm(gs(4*i-3:4*i-1,4) - p_obs');
%     end
%     c = r_obs+0.02-1*min(distances);
    c = [];
    lsqoptions = optimoptions('lsqlin','Display','off');
    magnifier = 1;
    ceq = Equilibrium_optim(S1,qul,uq, magnifier, lsqoptions);
end