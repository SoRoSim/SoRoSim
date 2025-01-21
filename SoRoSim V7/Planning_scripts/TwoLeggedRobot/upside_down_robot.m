clear
close all
load("upside_down_two_legged.mat");
S1.VLinks(1).E = 5.105e+7;
S1.VLinks(3).E = 5.105e+7;
S1.CVTwists{1}(2).UpdateAll;
S1.CVTwists{3}(2).UpdateAll;
S1 = S1.Update;
%%
[constraint_surface, root1, root2] = find_constraint(S1,qu_uq_l, 0.7);
%%

g_desired = [0.0000         0   1.0000    0.3
                0    1.0000         0         0
                -1.0000         0    0.0000    0.433
                0         0         0    1.0000];
% qu_uq_l0 = [qu_uq_l0; root1; root2];
qu_uq_l0 = zeros(80,1);
constraints_handle = @(qu_uq_l)constraints(S1, qu_uq_l, constraint_surface);
options = optimoptions('fmincon','Display','iter','OptimalityTolerance',1e-20,'StepTolerance',1e-20 ,'MaxFunctionEvaluations',2e10,'Algorithm', 'sqp');%,'EnableFeasibilityMode',true);%,'OptimalityTolerance',1e-10,'StepTolerance',1e-20);
lb(1:78) = -inf;
lb(79:80) = 0;
ub(1:78) = inf;
ub(79:80) = 1;

qu_uq_l = fmincon(@(qu_uq_l)objective_function(S1, qu_uq_l, g_desired), qu_uq_l0, [],[],[],[],lb,ub,constraints_handle,options);
%%
[c,ceq] = constraints(S1,qu_uq_l,constraint_surface);
function E = objective_function(S1, qu_uq_l, g_desired)
    q = qu_uq_l(1:S1.ndof);
    gs = S1.FwdKinematics(q);
    g_platform = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    E = norm(piecewise_logmap(ginv(g_desired)*g_platform));
end

function [c, ceq] = constraints(S1, qu_uq_l,constrain_surface)
    qul = [qu_uq_l(1:S1.ndof); qu_uq_l(S1.ndof+13:S1.ndof+18)];
    uq = qu_uq_l(S1.ndof+1:S1.ndof+12);
    xbar1 = qu_uq_l(S1.ndof+19);
    xbar2 = qu_uq_l(S1.ndof+20);
    c = [];
    lsqoptions = optimoptions('lsqlin','Display','off');
    magnifier = 1;
    eq = Equilibrium_optim(S1,qul,uq, magnifier, lsqoptions);
    Xs = S1.CVTwists{1}(2).Xs;
    V = [ones(length(Xs),1) Xs Xs.^2];
    g = S1.FwdKinematics(qu_uq_l(1:S1.ndof));

    x1 = g(4+1:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    y1 = g(4+2:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    z1 = g(4+3:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    poly_x = (V'*V)\V'*x1;
    poly_y = (V'*V)\V'*y1;
    poly_z = (V'*V)\V'*z1;
    xh1 = [poly_x'*[1; xbar1; xbar1^2] poly_y'*[1; xbar1; xbar1^2] poly_z'*[1; xbar1; xbar1^2]];

    Xs = S1.CVTwists{1}(2).Xs;
    V = [ones(length(Xs),1) Xs Xs.^2];

    x2 = g(4*(length(S1.CVTwists{1}(2).Xs) + 4)+1:4:4*(length(S1.CVTwists{1}(2).Xs)+length(S1.CVTwists{3}(2).Xs)+4),4);
    y2 = g(4*(length(S1.CVTwists{1}(2).Xs) + 4)+2:4:4*(length(S1.CVTwists{1}(2).Xs)+length(S1.CVTwists{3}(2).Xs)+4),4);
    z2 = g(4*(length(S1.CVTwists{1}(2).Xs) + 4)+3:4:4*(length(S1.CVTwists{1}(2).Xs)+length(S1.CVTwists{3}(2).Xs)+4),4);
    poly_x = (V'*V)\V'*x2;
    poly_y = (V'*V)\V'*y2;
    poly_z = (V'*V)\V'*z2;
    xh2 = [poly_x'*[1; xbar2; xbar2^2] poly_y'*[1; xbar2; xbar2^2] poly_z'*[1; xbar2; xbar2^2]];
    eq1 = norm([constrain_surface.hole_1 constrain_surface.height] - xh1);
    eq2 = norm([constrain_surface.hole_2 constrain_surface.height] - xh2);
    ceq = [eq; eq1; eq2];
end