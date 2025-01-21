%% path planning
clear
close all
load("two_legged_platform.mat");
S1.VLinks(1).E = 5.105e+8;
S1.VLinks(3).E = 5.105e+8;
S1.CVTwists{1}(2).UpdateAll;
S1.CVTwists{3}(2).UpdateAll;
S1 = S1.Update;

%%
g_des_final = [0.0000         0   -1.0000    0.25
                0    1.0000         0         0
                1.0000         0    0.0000    0.38
                0         0         0    1.0000];

g_platform_0 = [0.0000         0   -1.0000    0.25
                0    1.0000         0         0
                1.0000         0    0.0000    0.40
                0         0         0    1.0000];

ndof = S1.ndof;
n_x_t = ndof+18;
total_time = 1;
n_points = 10;
dt = total_time/(n_points-1);
%find q_0 for all time
qu_uq_l0 = zeros(n_points,S1.ndof+18);

constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l, n_x_t,n_points, g_des_final,g_platform_0);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-10 ,'MaxFunctionEvaluations',2e10, 'DiffMinChange', 1e-5,'SpecifyObjectiveGradient',true);%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);

qu_uq_l = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l,n_points), qu_uq_l0, [],[],[],[],[],[],constraints_handle,options);

%%
E = objective_function(S1, qu_uq_l)
[c, c_eq] = constraints(S1, qu_uq_l,n_x_t, n_points, g_des_final, g_platform_0)

%% plot the result
figure
S1.PlotParameters.ClosePrevious= false;
qs = qu_uq_l(:,1:S1.ndof);
gs1 = S1.FwdKinematics(qs(1,:));
for i = 1:5
S1.plotq(qs(2*i,:))
hold on
end
plotTransforms(se3(g_des_final),'FrameSize',0.08)


function [E, gradE] = objective_function(S1, qu_uq_l,N)
    ndof = S1.ndof;
    dxdt = diff(qu_uq_l(:,1:S1.ndof));
    E = sum(sum(dxdt.^2, 2));

    gradE = zeros(N, length(qu_uq_l));

    % Loop through each degree of freedom
    for j = 1:ndof
        % Interior points
        gradE(2:N-1, j) = ...
            2 * (qu_uq_l(2:N-1, j) - qu_uq_l(1:N-2, j)) - ...
            2 * (qu_uq_l(3:N, j) - qu_uq_l(2:N-1, j));

        % Boundary points
        gradE(1, j) = -2 * (qu_uq_l(2, j) - qu_uq_l(1, j));
        gradE(N, j) = 2 * (qu_uq_l(N, j) - qu_uq_l(N-1, j));
    end

    % Flatten the gradient into a vector
    gradE = gradE(:);

end

function [c, ceq] = constraints(S1, qu_uq_l, n_x_t, n_points, g_desired,g_0)
    
    lsqoptions = optimoptions('lsqlin','Display','off');
    magnifier = 1;
    eq_1= [];
    for i = 1:n_points
        eq_1 = [eq_1; Equilibrium_optim(S1,[qu_uq_l(i,1:S1.ndof)'; qu_uq_l(i,S1.ndof+13:end)'],qu_uq_l(i,S1.ndof+1:S1.ndof+12)', magnifier, lsqoptions)];
    end
    qT = qu_uq_l(end,1:S1.ndof);
    gs = S1.FwdKinematics(qT);
    g_platformT = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    eq_2 = piecewise_logmap(ginv(g_platformT)*g_desired);
    q0 = qu_uq_l(1,1:S1.ndof);
    gs = S1.FwdKinematics(q0);
    g_platform0 = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    eq_3 = piecewise_logmap(ginv(g_platform0)*g_0);
    c = [];
    
    ceq = [eq_1;eq_2;eq_3];

end