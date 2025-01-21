clear
close all
load("upside_down_two_legged.mat")
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(3).E = 5.105e+7;
S1.CVTwists{1}(2).UpdateAll;
S1.CVTwists{3}(2).UpdateAll;
S1 = S1.Update;
%%
[constraint_surface, root1, root2] = find_constraint(S1,qu_uq_l_con(1,:), -0.1);


%%
g_des_final = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.433
                0         0         0    1.0000];

g_platform_0 = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    -0.35
                0         0         0    1.0000];
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

constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l, n_points, constraint_surface, g_platform_0);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5,'Algorithm','sqp');%EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

qu_uq_l_con = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l,g_des_final), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
%%
figure
S1.PlotParameters.ClosePrevious= false;
qs = qu_uq_l_con(:,1:S1.ndof);
gs1 = S1.FwdKinematics(qs(1,:));
plot_constraint_surface(constraint_surface)
for i = 1:5
    S1.plotq(qs(2*i,:))
    hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)


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

plot_constraint_surface(constrain_surface)

%%
q1 = qu_uq_l(1,1:S1.ndof);
g = S1.FwdKinematics(q1);
xs = g(11*4+1:4:10*4+1+4*length(S1.CVTwists{3}(2).Xs),4);
ys = g(11*4+2:4:10*4+2+4*length(S1.CVTwists{3}(2).Xs),4);
zs = g(11*4+3:4:10*4+3+4*length(S1.CVTwists{3}(2).Xs),4);

plot3(xs,ys, zs,'*')
%%
X = polyfit(S1.CVTwists{3}(2).Xs,xs,4);
Y = polyfit(S1.CVTwists{3}(2).Xs,ys,4);
Z = polyfit(S1.CVTwists{3}(2).Xs, zs, 4);

Xbars = linspace(0,1,10);

Xs = polyval(X,Xbars);
Ys = polyval(Y,Xbars);
Zs = polyval(Z,Xbars);
plot3(Xs, Ys, Zs)
%%
E = objective_function(S1, qu_uq_l0)
[c, ceq] = constraints(S1, qu_uq_l, n_points, constrain_surface,g_des_final, g_platform_0)


function E = objective_function(S1, qu_uq_l,g_desired)
    dxdt = diff(qu_uq_l(:,1:S1.ndof));
    qT = qu_uq_l(end,1:S1.ndof);
    gs = S1.FwdKinematics(qT);
    g_platformT = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    E = sum(sum(dxdt.^2, 2)) + norm(piecewise_logmap(ginv(g_desired)*g_platformT));
end

function [c, ceq] = constraints(S1, qu_uq_l,n_points,constrain_surface, g_0)
    c = [];
    qul = [qu_uq_l(:,1:S1.ndof)'; qu_uq_l(:,S1.ndof+13:S1.ndof+18)']';
    uq = qu_uq_l(:,S1.ndof+1:S1.ndof+12);
    x1 = qu_uq_l(:,S1.ndof+19);
    x2 = qu_uq_l(:,S1.ndof+20);
    ceq = [];
    for i =1:n_points
        g = S1.FwdKinematics(qu_uq_l(i,1:S1.ndof));
        X1s = S1.CVTwists{1}(2).Xs;
        X2s = S1.CVTwists{3}(2).Xs;
        X1 = polyfit(X1s, g(5:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
        Y1 = polyfit(X1s, g(6:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
        Z1 = polyfit(X1s, g(7:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
        xh1 = [polyval(X1,x1(i)) polyval(Y1,x1(i)) polyval(Z1,x1(i))];
        eq1 = norm([constrain_surface.hole_1 constrain_surface.height] - xh1);
        X2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+1:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
        Y2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+2:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
        Z2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+3:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
        xh2 = [polyval(X2,x2(i)) polyval(Y2,x2(i)) polyval(Z2,x2(i))];
        eq2 = norm([constrain_surface.hole_2 constrain_surface.height] - xh2);
        lsqoptions = optimoptions('lsqlin','Display','off');
        magnifier = 1;
        eq3 = Equilibrium_optim(S1,qul(i,:)',uq(i,:)', magnifier, lsqoptions);
        ceq = [ceq; eq3];
        c = [;eq1-0.05;eq2-0.05];
    end
    
    q0 = qu_uq_l(1,1:S1.ndof);
    gs = S1.FwdKinematics(q0);
    g_platform0 = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    eq_5 = piecewise_logmap(ginv(g_platform0)*g_0);
    ceq = [ceq; eq_5];
end

function plot_constraint_surface(constrain_surface)
   radius = constrain_surface.radius;
   theta = linspace(0, 2*pi);
   z = ones(1,length(theta))*constrain_surface.height;
   x1 = radius*cos(theta) + constrain_surface.hole_1(1);
   y1 = radius*sin(theta) +constrain_surface.hole_1(2);
   x2 = radius*cos(theta) + constrain_surface.hole_2(1);
   y2 = radius*sin(theta) + constrain_surface.hole_2(2);
   plot3(x1, y1,z, 'r');
   hold on 
   plot3(x2,y2,z,'r');
end