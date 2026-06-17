clear; close all;
robot = loadrobot("universalUR10");
robot.DataFormat = 'column';
load("UR_10_linkage.mat");

%% ===== Build robot collision bodies =====
robotbodies = SorosimContactBody.empty(0,0);
contact_body_counter = 1;

opts_multi = vhacdOptions("IndividualMesh","MaxNumConvexHulls",3, ...
    "MaxNumVerticesPerHull",30,"VoxelResolution",128000);
[robotbodies, contact_body_counter] = addRobotBodies(robot, 4:5, ...
    opts_multi, robotbodies, contact_body_counter, false);

opts_single = vhacdOptions("IndividualMesh","MaxNumConvexHulls",1, ...
    "MaxNumVerticesPerHull",30,"VoxelResolution",128000);
[robotbodies, contact_body_counter] = addRobotBodies(robot, 6, ...
    opts_single, robotbodies, contact_body_counter, false);

[robotbodies, contact_body_counter] = addCombinedRobotBody(robot, 7:8, 7, ...
    opts_single, robotbodies, contact_body_counter);

%% ===== Build environment =====
EnvironmentBodies = buildSimpleEnvironment();

%% ===== Endpoint poses =====
q_start_seed = [ 0.4 -0.4 0.3 0 0 0]';
q_end_seed   = [-0.6 -0.3 0.3 0 0 0]';
g_init  = getTransform(robot, q_start_seed, "ee_link");
g_final = getTransform(robot, q_end_seed,   "ee_link");

ik = inverseKinematics('RigidBodyTree', robot, ...
    'SolverAlgorithm','BFGSGradientProjection');
weights = [1 1 1 1 1 1];

[q_start, info_s] = ik('ee_link', g_init,  weights, q_start_seed);
[q_end,   info_e] = ik('ee_link', g_final, weights, q_end_seed);

assert(info_s.ExitFlag == 1 && info_s.PoseErrorNorm < 1e-6, ...
    'IK failed at start: %s, err=%.3e', info_s.Status, info_s.PoseErrorNorm);
assert(info_e.ExitFlag == 1 && info_e.PoseErrorNorm < 1e-6, ...
    'IK failed at end: %s, err=%.3e',   info_e.Status, info_e.PoseErrorNorm);

q_start = q_start(:);
q_end   = q_end(:);

%% ===== Build collision pairs =====
nP = numel(robotbodies) * numel(EnvironmentBodies);
Pairs = SorosimContactPair.empty(nP, 0);
idx = 1;
for i = 1:numel(robotbodies)
    for j = 1:numel(EnvironmentBodies)
        Pairs(idx) = SorosimContactPair(robotbodies(i), EnvironmentBodies(j), idx);
        Pairs(idx).use_warmstart = false;
        idx = idx + 1;
    end
end

%% ===== Problem setup =====
prob.N         = 10;
prob.q_init    = q_start;
prob.q_des     = q_end;
prob.Linkage   = robot;
prob.Pairs     = Pairs;
prob.w_smooth  = 1;
prob.w_length  = 1;
prob.eps_clear = 5e-2;
prob.dmax_step = 0.35;


% ===== RRT initial guess =====
rrt.maxIter   = 5000;
rrt.stepSize = 0.30;     % closer to prob.dmax_step
rrt.goalBias = 0.30;
rrt.goalTol  = 0.50;
margin = 1.0;
rrt.lb = min(q_start, q_end) - margin;
rrt.ub = max(q_start, q_end) + margin;

fprintf('q_start feasible: %d\n', config_feasible(q_start, prob));
fprintf('q_end   feasible: %d\n', config_feasible(q_end,   prob));

[path_rrt, ok] = rrt_plan(q_start, q_end, prob, rrt);
assert(ok, 'RRT failed to find a path');

% Resample to exactly prob.N waypoints
q_path = resample_path(path_rrt, prob.N);
x0     = q_path(:);

% Sanity check
[c0, ~] = nl_con(x0, prob);
fprintf('RRT init: max(c) = %.3e, num violated = %d / %d\n', ...
        max(c0), sum(c0 > 0), numel(c0));

% %% ===== Visualize RRT solution =====
% q_full = path_rrt;                    % already 6 x M from rrt_plan
% nC = size(q_full, 2);
% 
% figure('Color','w'); hold on;
% ax = gca;
% 
% for i = 1:nC
%     show(robot, q_full(:,i), ...
%         'Frames','off', ...
%         'PreservePlot', true, ...
%         'FastUpdate',   false, ...
%         'Visuals',      'on', ...
%         'Parent',       ax);
% end
% 
% patches = findall(ax, 'Type', 'patch');
% nP = numel(patches);
% for k = 1:nP
%     t = (k-1) / max(nP-1, 1);
%     set(patches(k), ...
%         'FaceAlpha', 0.25, ...
%         'EdgeColor', 'none', ...
%         'FaceColor', [1-t, 0.3, t]);
% end
% 
% for i = 1:numel(EnvironmentBodies)
%     EnvironmentBodies(i).attachHandles(ax);
%     EnvironmentBodies(i).setPose(eye(4));
% end
% 
% axis tight; axis equal; grid on; view(45, 25);
% camlight headlight; lighting gouraud; material dull;
% xlabel('x'); ylabel('y'); zlabel('z');
% title(sprintf('RRT path: %d nodes', nC));

% ===== traj opt =====
useGrads = true;
opts = optimoptions('fmincon', ...
    'Algorithm',                 'interior-point', ...
    'Display',                   'iter', ...
    'MaxFunctionEvaluations',    2e5, ...
    'MaxIterations',             1000, ...
    'SpecifyObjectiveGradient',  useGrads, ...
    'SpecifyConstraintGradient', useGrads, ...
    'EnableFeasibilityMode',     false, ...
    'ConstraintTolerance',       1e-6, ...
    'OptimalityTolerance',       1e-4, ...
    'StepTolerance',             1e-6, ...
    'ScaleProblem',              true);

solveCfg = struct('trySQPOnWarning', true, ...
                  'quietSingularWarnings', true, ...
                  'feasMarginOK', 1e-4);
tic
profile on
sol = solve_kinematic_fmincon(prob, x0, [], [], opts, solveCfg, useGrads);
profile off
toc
profile viewer

save("solution");

%% ===== Visualize result =====
q_full = [prob.q_init, reshape(sol.x, 6, prob.N), prob.q_des];
nC = size(q_full, 2);

figure('Color','w'); hold on;
ax = gca;

% Draw all configurations, keeping each one
for i = 1:nC
    show(robot, q_full(:,i), ...
        'Frames','off', ...
        'PreservePlot', true, ...
        'FastUpdate',   false, ...
        'Visuals',      'on', ...
        'Parent',       ax);
end

% Make the meshes semi-transparent and color-graded along the path
patches = findall(ax, 'Type', 'patch');
nP = numel(patches);
for k = 1:nP
    % oldest patches (drawn first = q_init end) at one color, newest at another
    t = (k-1) / max(nP-1, 1);
    set(patches(k), ...
        'FaceAlpha', 0.25, ...
        'EdgeColor', 'none', ...
        'FaceColor', [1-t, 0.3, t]);   % red -> blue along trajectory
end

% Environment
for i = 1:numel(EnvironmentBodies)
    EnvironmentBodies(i).attachHandles(ax);
    EnvironmentBodies(i).setPose(eye(4));
end

axis tight
axis equal; grid on; view(45, 25);
camlight headlight; lighting gouraud; material dull;
xlabel('x'); ylabel('y'); zlabel('z');
title(sprintf('Trajectory: %d configurations', nC));


%% ===== Visualize RRT vs trajectory-optimization solution =====
q_rrt = q_path;                                                     % 6 x N (RRT, resampled)
q_opt = [prob.q_init, reshape(sol.x, 6, prob.N), prob.q_des];       % 6 x (N+2)

figure('Color','w'); hold on;
ax = gca;

% --- Draw optimization configs ---
nO = size(q_opt, 2);
for i = 1:nO
    show(robot, q_opt(:,i), ...
        'Frames','off','PreservePlot',true,'FastUpdate',false, ...
        'Visuals','on','Parent',ax);
end

patches_opt = findall(ax, 'Type', 'patch');

% --- Color optimization (blue, alpha ramp) ---
patches_opt = flipud(patches_opt(:));
nPO = numel(patches_opt);
for k = 1:nPO
    t = (k-1) / max(nPO-1, 1);
    set(patches_opt(k), ...
        'FaceColor', [1.0 0.5 0.0], ...
        'FaceAlpha', 0.30 + 0.5*t, ...
        'EdgeColor', 'none');
end

% --- EE traces ---
nR = size(q_rrt, 2);
ee_rrt = zeros(3, nR);
for i = 1:nR
    g = getTransform(robot, q_rrt(:,i), "ee_link");
    ee_rrt(:,i) = g(1:3,4);
end
ee_opt = zeros(3, nO);
for i = 1:nO
    g = getTransform(robot, q_opt(:,i), "ee_link");
    ee_opt(:,i) = g(1:3,4);
end
% plot3(ax, ee_rrt(1,:), ee_rrt(2,:), ee_rrt(3,:), '-o-', 'Color','b','LineWidth',3);
% plot3(ax, ee_opt(1,:), ee_opt(2,:), ee_opt(3,:), '-o-', 'Color','r','LineWidth',3);
plot3(ax, ee_rrt(1,:), ee_rrt(2,:), ee_rrt(3,:), '-o', ...
    'Color','b','LineWidth',3,'MarkerSize',8, ...
    'MarkerFaceColor','none','MarkerEdgeColor','b');

plot3(ax, ee_opt(1,:), ee_opt(2,:), ee_opt(3,:), '-o', ...
    'Color','r','LineWidth',3,'MarkerSize',8, ...
    'MarkerFaceColor','none','MarkerEdgeColor','r');

% --- Start / end EE frames ---
drawFrame(getTransform(robot, prob.q_init, "ee_link"), 'framesize', 0.2, 'linewidth', 3);
drawFrame(getTransform(robot, prob.q_des,  "ee_link"), 'framesize', 0.2, 'linewidth', 3);

% --- Environment (solid, opaque) ---
for i = 1:numel(EnvironmentBodies)
    EnvironmentBodies(i).attachHandles(ax);
    EnvironmentBodies(i).setPose(eye(4));
end
env_patches = findall(ax, 'Type', 'patch');
env_patches = setdiff(env_patches, patches_opt, 'stable');
for k = 1:numel(env_patches)
    set(env_patches(k), ...
        'FaceColor', [0.55 0.55 0.60], ...
        'FaceAlpha', 1.0, ...
        'EdgeColor', 'none', ...
        'LineWidth', 0.3);
end

axis tight; axis equal; grid off; view(45, 25);
camlight headlight; lighting gouraud; material dull;
xlabel('x'); ylabel('y'); zlabel('z');


% size the plane from current axes (×2 to feel "infinite")
ax2  = axis;
R   = max([diff(ax2(1:2)), diff(ax2(3:4)), diff(ax2(5:6))]);
S   = 2*R;
pitch = R/20;

cDark  = [0.15 0.25 0.45];
cLight = [0.25 0.45 0.75];

p0 = [0 0 0];
n  = [0 0 1];
n  = n / norm(n);

if abs(n(1)) < 0.9, tmp = [1 0 0]; else, tmp = [0 1 0]; end
u = cross(n, tmp);  u = u / max(norm(u), eps);
v = cross(n, u);    v = v / max(norm(v), eps);

s = linspace(-S, S, 81);
tp = linspace(-S, S, 81);
[Sg, Tg] = meshgrid(s, tp);

X = p0(1) + Sg.*u(1) + Tg.*v(1);
Y = p0(2) + Sg.*u(2) + Tg.*v(2);
Z = p0(3) + Sg.*u(3) + Tg.*v(3);

chk = mod(round(Sg/pitch) + round(Tg/pitch), 2);
C = zeros([size(chk), 3]);
for k = 1:3
    C(:,:,k) = cDark(k).*(chk==0) + cLight(k).*(chk==1);
end

srf = surf(X, Y, Z, C, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
srf.SpecularStrength = 0.9;
srf.DiffuseStrength  = 0.7;
srf.AmbientStrength  = 0.5;

xlim([-0.5 1.5])
ylim([-1 1])
zlim([0 1.5])

xlabel(ax,'$x$','Interpreter','latex','FontSize',24);
ylabel(ax,'$y$','Interpreter','latex','FontSize',24);
zlabel(ax,'$z$','Interpreter','latex','FontSize',24);

set(ax, ...
    'Units','normalized', ...
    'Position',[0.02 0.02 0.96 0.96], ...
    'OuterPosition',[0.00 0.00 1.00 1.00], ...
    'LooseInset',[0 0 0 0], ...
    'FontSize',24, ...
    'TickLabelInterpreter','latex');
%% Animation: RRT + optimized trajectory, save MP4

videoName = 'ur10_rrt_vs_opt.mp4';
fps = 30;
interpPerSeg = 15;
holdSeconds  = 2;
rotDegPerSec = 12;

eeName = "ee_link";

v = VideoWriter(videoName, 'MPEG-4');
v.FrameRate = fps;
v.Quality = 100;
open(v);

% Global LaTeX font settings
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% Figure and axes
fig = figure('Color','w', ...
    'Units','pixels');

ax = axes(fig);
hold(ax,'on');

set(ax, ...
    'Units','normalized', ...
    'Position',[0.02 0.02 0.96 0.96], ...
    'OuterPosition',[0.00 0.00 1.00 1.00], ...
    'LooseInset',[0 0 0 0], ...
    'FontSize',18, ...
    'TickLabelInterpreter','latex');

axis(ax,'manual');
axis(ax,'vis3d');
daspect(ax,[1 1 1]);

xlim(ax,[-0.5 1.5]);
ylim(ax,[-1 1]);
zlim(ax,[0 1.5]);

camTargetFixed = [0.5 0 0.75];
elFixed        = 25;
azStart        = 45;

view(ax,azStart,elFixed);
camtarget(ax,camTargetFixed);
camva(ax,11);

grid(ax,'off');
box(ax,'on');

xlabel(ax,'$x$','Interpreter','latex','FontSize',22);
ylabel(ax,'$y$','Interpreter','latex','FontSize',22);
zlabel(ax,'$z$','Interpreter','latex','FontSize',22);

camlight(ax,'headlight');
lighting(ax,'gouraud');
material(ax,'dull');

% Precompute checker plane
axlim = [-0.5 1.5 -1 1 -0.25 1.5];
R = max([diff(axlim(1:2)), diff(axlim(3:4)), diff(axlim(5:6))]);
S = 2*R;
pitch = R/20;

cDark  = [0.15 0.25 0.45];
cLight = [0.25 0.45 0.75];

p0 = [0 0 0];
n  = [0 0 1]; n = n/norm(n);

if abs(n(1)) < 0.9, tmp = [1 0 0]; else, tmp = [0 1 0]; end
u = cross(n,tmp); u = u/max(norm(u),eps);
w = cross(n,u);   w = w/max(norm(w),eps);

s = linspace(-S,S,81);
tp = linspace(-S,S,81);
[Sg,Tg] = meshgrid(s,tp);

X = p0(1) + Sg.*u(1) + Tg.*w(1);
Y = p0(2) + Sg.*u(2) + Tg.*w(2);
Z = p0(3) + Sg.*u(3) + Tg.*w(3);

chk = mod(round(Sg/pitch) + round(Tg/pitch),2);
C = zeros([size(chk),3]);
for k = 1:3
    C(:,:,k) = cDark(k).*(chk==0) + cLight(k).*(chk==1);
end

% Robot color
robotOrange = [1.0 0.5 0.0];

% Fixed EE frames at start/end (computed once)
T_start = getTransform(robot, prob.q_init, eeName);
T_end   = getTransform(robot, prob.q_des,  eeName);

% Precompute EE traces
nR = size(q_rrt,2);
nO = size(q_opt,2);

ee_rrt = zeros(3,nR);
for i = 1:nR
    g = getTransform(robot,q_rrt(:,i),eeName);
    ee_rrt(:,i) = g(1:3,4);
end

% Interpolate q_opt in joint space
nDense = (nO-1)*interpPerSeg + 1;
tSrc   = 1:nO;
tDense = linspace(1, nO, nDense);
q_dense = zeros(6, nDense);
for j = 1:6
    q_dense(j,:) = interp1(tSrc, q_opt(j,:), tDense, 'linear');
end

ee_dense = zeros(3,nDense);
T_dense  = cell(1,nDense);
for i = 1:nDense
    g = getTransform(robot,q_dense(:,i),eeName);
    T_dense{i}    = g;
    ee_dense(:,i) = g(1:3,4);
end

nHold = round(holdSeconds * fps);
nTotal = nDense + nHold;

% Animation loop
for i = 1:nTotal

    if i <= nDense
        qi   = q_dense(:,i);
        Tcur = T_dense{i};
        kEnd = i;
    else
        qi   = q_dense(:,end);
        Tcur = T_dense{end};
        kEnd = nDense;
    end

    azNow = azStart + rotDegPerSec * (i-1) / fps;

    cla(ax);
    hold(ax,'on');

    % Environment
    for j = 1:numel(EnvironmentBodies)
        EnvironmentBodies(j).attachHandles(ax);
        EnvironmentBodies(j).setPose(eye(4));
    end
    env_patches = findall(ax,'Type','patch');
    for k = 1:numel(env_patches)
        set(env_patches(k), ...
            'FaceColor',[0.55 0.55 0.60], ...
            'FaceAlpha',1.0, ...
            'EdgeColor','none');
    end

    % Checker plane
    srf = surf(ax,X,Y,Z,C, ...
        'EdgeColor','none', ...
        'FaceAlpha',0.5);
    srf.SpecularStrength = 0.9;
    srf.DiffuseStrength  = 0.7;
    srf.AmbientStrength  = 0.5;

    % RRT trace (full)
    plot3(ax,ee_rrt(1,:),ee_rrt(2,:),ee_rrt(3,:), ...
        '--','Color',[0.85 0.10 0.10],'LineWidth',2.0);

    % Optimized trace up to current frame
    plot3(ax,ee_dense(1,1:kEnd),ee_dense(2,1:kEnd),ee_dense(3,1:kEnd), ...
        '-','Color','b','LineWidth',2.0);

    % Robot
    show(robot,qi, ...
        'Frames','off', ...
        'PreservePlot',true, ...
        'FastUpdate',false, ...
        'Visuals','on', ...
        'Parent',ax);

    delete(findall(fig, 'Tag', 'CornerCoordinateFrame'));

    all_patches = findall(ax,'Type','patch');
    robot_patches = setdiff(all_patches, env_patches, 'stable');
    for k = 1:numel(robot_patches)
        set(robot_patches(k), ...
            'FaceColor', robotOrange, ...
            'FaceAlpha', 1.0, ...
            'EdgeColor', 'none');
    end

    % Start, end, and current EE frames
    drawFrame(T_start, 'framesize', 0.2, 'linewidth', 3);
    drawFrame(T_end,   'framesize', 0.2, 'linewidth', 3);
    drawFrame(Tcur,    'framesize', 0.15,'linewidth', 2);

    % Camera and limits
    xlim(ax,[-0.5 1.5]);
    ylim(ax,[-1 1]);
    zlim(ax,[0 1.5]);

    axis(ax,'manual');
    axis(ax,'vis3d');
    daspect(ax,[1 1 1]);

    view(ax, azNow, elFixed);
    camtarget(ax, camTargetFixed);
    camva(ax,11);

    grid(ax,'off');
    box(ax,'on');

    xlabel(ax,'$x$','Interpreter','latex','FontSize',22);
    ylabel(ax,'$y$','Interpreter','latex','FontSize',22);
    zlabel(ax,'$z$','Interpreter','latex','FontSize',22);

    set(ax, ...
        'Units','normalized', ...
        'Position',[0.02 0.02 0.96 0.96], ...
        'OuterPosition',[0.00 0.00 1.00 1.00], ...
        'LooseInset',[0 0 0 0], ...
        'FontSize',18, ...
        'TickLabelInterpreter','latex');

    camlight(ax,'headlight');
    lighting(ax,'gouraud');
    material(ax,'dull');

    drawnow;
    frame = getframe(fig);
    writeVideo(v,frame);
end

close(v);

fprintf('Saved animation to %s\n', videoName);
%%
% ====================================================================
% Local functions
% ====================================================================

function [bodies, counter] = addRobotBodies(robot, bodyIdx, opts, bodies, counter, attach_at_eye)
    for k = bodyIdx
        currBody = robot.Bodies{k};
        clearCollision(currBody);
        vizData = getVisual(currBody);
        if isempty(vizData), continue; end

        collisionArray = collisionVHACD(vizData(1).Triangulation, opts);
        g = getTransform(robot, robot.homeConfiguration, currBody.Name);

        for i = 1:numel(collisionArray)
            V = collisionArray{i}.Vertices;
            K = convhull(V);
            Vhull = V(unique(K(:)), :);
            [A, c] = MinVolEllipse(Vhull', 1e-1, 1.1);

            cb = OurFormEllipsoid(A, c);
            cb.id = counter;
            cb.bodyName = currBody.Name;
            bodies(counter) = cb;
            counter = counter + 1;

            cb.attachHandles(gca);
            if attach_at_eye
                cb.setPose(eye(4));
            else
                cb.setPose(g);
            end
        end
        for j = 1:numel(collisionArray)
            addCollision(currBody, collisionArray{j});
        end
    end
end

function [bodies, counter] = addCombinedRobotBody(robot, bodyIdx, attachIdx, opts, bodies, counter)
    V = [];
    for k = bodyIdx
        currBody = robot.Bodies{k};
        clearCollision(currBody);
        vizData = getVisual(currBody);
        if isempty(vizData), continue; end
        collisionArray = collisionVHACD(vizData(1).Triangulation, opts);
        for i = 1:numel(collisionArray)
            V = [V; collisionArray{i}.Vertices]; %#ok<AGROW>
        end
    end
    if isempty(V), return; end

    K = convhull(V);
    Vhull = V(unique(K(:)), :);
    [A, c] = MinVolEllipse(Vhull', 1e-1, 1.1);

    cb = OurFormEllipsoid(A, c);
    cb.id = counter;
    cb.bodyName = robot.Bodies{attachIdx}.Name;
    bodies(counter) = cb;
    counter = counter + 1;

    cb.attachHandles(gca);
    cb.setPose(eye(4));
end

function E = buildSimpleEnvironment()
    g_center = eul2tform([0 -pi/2 0]);
    g_center(1:3,4) = [1; 0; 0.25];
    beta = 20;

    A_box = [eye(3); -eye(3)];
    bA = 0.5/2 * ones(6,1);
    p1 = idcol_make_params(2, beta, A_box, bA);
    E(1) = SorosimContactBody(1, 2, p1);
    E(1).g_JC = g_center;

    p2 = idcol_make_params(3, beta, 0.15, 0.15, 0.1, 0.1);
    E(2) = SorosimContactBody(2, 3, p2);
    g_cyl = g_center; g_cyl(3,4) = g_center(3,4) + 0.35;
    E(2).g_JC = g_cyl;

    p3 = idcol_make_params(3, beta, 0.1, 0.1, 0.25, 0.35);
    E(3) = SorosimContactBody(3, 3, p3);
    g_cyl = g_center; g_cyl(1,4) = g_cyl(1,4) - 0.35; g_cyl(2,4) = g_cyl(2,4) + 0.1;
    E(3).g_JC = g_cyl;
end

function contact_body = OurFormEllipsoid(A, center)
    [Q, lambda] = eig(A);
    lambda = diag(lambda);
    [lambda, idx] = sort(lambda, 'descend');
    Q = Q(:, idx);
    if det(Q) < 0, Q(:,3) = -Q(:,3); end
    if max(lambda)/min(lambda) < 1.05, Q = eye(3); end
    axes_lengths = 1 ./ sqrt(lambda);
    g = rotm2tform(Q);
    g(1:3,4) = center;
    a = axes_lengths(1); b = axes_lengths(2); c = axes_lengths(3);
    params = idcol_make_params(4, 1, a, b, c);
    contact_body = SorosimContactBody(1, 4, params);
    contact_body.g_JC = g;
end

function sol = solve_kinematic_fmincon(prob, x0, lb, ub, opts, cfg, useGrads) %#ok<INUSD>
    if nargin < 6 || isempty(cfg), cfg = struct(); end
    if ~isfield(cfg,'trySQPOnWarning'),        cfg.trySQPOnWarning = true; end
    if ~isfield(cfg,'quietSingularWarnings'),  cfg.quietSingularWarnings = true; end
    if ~isfield(cfg,'feasMarginOK'),           cfg.feasMarginOK = 1e-4; end

    [x_end, fval, exitflag, out, singularWarn] = run_once(prob, x0, lb, ub, opts, cfg.quietSingularWarnings);

    x = pick_bestfeasible(x_end, out);
    [c_best, ~] = nl_con(x, prob);
    max_c = max(c_best);

    sol = pack_solution(x, fval, exitflag, out, opts.Algorithm, "primary", max_c, singularWarn);

    if max_c <= -cfg.feasMarginOK
        sol.status = "ok";
        return;
    end

    if cfg.trySQPOnWarning && (singularWarn || max_c > -cfg.feasMarginOK)
        fprintf('[solve] Switching to SQP fallback (singularWarn=%d, max(c)=%.3e).\n', singularWarn, max_c);
        opts2 = optimoptions(opts, 'Algorithm','sqp', 'Display','final');
        [x_end2, fval2, exitflag2, out2, singularWarn2] = run_once(prob, x, lb, ub, opts2, cfg.quietSingularWarnings);

        x2 = pick_bestfeasible(x_end2, out2);
        [c2, ~] = nl_con(x2, prob);
        max_c2 = max(c2);

        sol2 = pack_solution(x2, fval2, exitflag2, out2, 'sqp', "fallback_sqp", max_c2, singularWarn2);

        if (max_c2 < max_c) || (max_c2 <= 0 && max_c > 0) || (abs(max_c2-max_c) < 1e-12 && fval2 < fval)
            sol = sol2;
            sol.status = "ok_fallback_sqp";
        else
            sol.status = "ok_primary";
        end
    else
        sol.status = "ok_weak_margin";
    end

    if sol.max_c > 0
        warning('Optimization returned infeasible solution: max(c)=%.3e', sol.max_c);
    end
end

function [x_end, fval, exitflag, out, singularWarn] = run_once(prob, x0, lb, ub, opts, quietSingularWarnings)
    singularWarn = false;
    ids = {'MATLAB:nearlySingularMatrix','MATLAB:singularMatrix','MATLAB:illConditionedMatrix'};
    old = cell(size(ids));
    for i = 1:numel(ids)
        old{i} = warning('query', ids{i});
        if quietSingularWarnings, warning('off', ids{i});
        else,                     warning('on',  ids{i}); end
    end
    lastwarn('');

    obj = @(x) obj_fun(x, prob);
    con = @(x) nl_con(x, prob);

    [x_end, fval, exitflag, out] = fmincon(obj, x0, [],[],[],[], lb, ub, con, opts);

    [~, wid] = lastwarn;
    if any(strcmp(wid, ids)), singularWarn = true; end

    for i = 1:numel(ids)
        warning(old{i}.state, ids{i});
    end
end

function x = pick_bestfeasible(x_end, out)
    x = x_end;
    if isfield(out,'bestfeasible') && isstruct(out.bestfeasible) && ...
            isfield(out.bestfeasible,'x') && ~isempty(out.bestfeasible.x)
        x = out.bestfeasible.x;
    end
end

function sol = pack_solution(x, fval, exitflag, out, alg, tag, max_c, singularWarn)
    sol = struct();
    sol.x = x;
    sol.fval = fval;
    sol.exitflag = exitflag;
    sol.out = out;
    sol.alg = alg;
    sol.tag = tag;
    sol.max_c = max_c;
    sol.singularWarn = singularWarn;
    sol.status = "unset";
end

function [J, dJdx] = obj_fun(x, prob)
    N = prob.N;
    Popt = reshape(x, 6, N);
    P = [prob.q_init, Popt, prob.q_des];

    J_s = 0; dJs = zeros(size(P));
    for k = 2:N+1
        a = P(:,k+1) - 2*P(:,k) + P(:,k-1);
        J_s = J_s + a.'*a;
        dJs(:,k+1) = dJs(:,k+1) + 2*a;
        dJs(:,k)   = dJs(:,k)   - 4*a;
        dJs(:,k-1) = dJs(:,k-1) + 2*a;
    end

    J_l = 0; dJl = zeros(size(P));
    for k = 1:N+1
        b = P(:,k+1) - P(:,k);
        J_l = J_l + b.'*b;
        dJl(:,k+1) = dJl(:,k+1) + 2*b;
        dJl(:,k)   = dJl(:,k)   - 2*b;
    end

    J = prob.w_smooth*J_s + prob.w_length*J_l;
    dJ_full = prob.w_smooth*dJs + prob.w_length*dJl;
    dJdx = dJ_full(:, 2:N+1);
    dJdx = dJdx(:);
end

function [c, ceq, dc, dceq] = nl_con(x, prob)
    N     = prob.N;
    robot = prob.Linkage;
    Pairs = prob.Pairs;
    ncp   = numel(Pairs);

    P    = reshape(x, 6, N);
    ceq  = [];
    dceq = [];

    c  = zeros((ncp+1)*N, 1);
    dc = zeros(6*N, (ncp+1)*N);

    for i = 1:N
        g = getTransform(robot, P(:,i), "ee_link");
        J = geometricJacobian(robot, P(:,i), "ee_link");
        c(i) = -g(3,4);
        dc(6*i-5:6*i, i) = -J(6,:).';
    end

    idx = N;
    for k = 1:N
        pk  = P(:,k);
        row = (6*(k-1)+1):(6*k);

        for icp = 1:ncp
            idx = idx + 1;

            g1  = getTransform(robot, pk, Pairs(icp).body1.bodyName);
            g2  = eye(4);
            out = Pairs(icp).solveContact(g1, g2);

            c(idx) = -out.alpha + 1 + prob.eps_clear;

            J1  = geometricJacobian(robot, pk, Pairs(icp).body1.bodyName);
            R1  = g1(1:3,1:3);
            J1  = [R1.' * J1(1:3,:); R1.' * J1(4:6,:)];
            J1  = dinamico_Adjoint(ginv(Pairs(icp).body1.g_JC)) * J1;

            g12 = Pairs(icp).get_relative(g1, g2);
            R12 = g12(1:3,1:3);
            r12 = g12(1:3,4);
            J12 = -dinamico_Adjoint(ginv(g12)) * J1;

            alpha   = out.alpha;
            lambda2 = out.lambda2;
            y       = R12.' * (out.x - r12) / alpha;
            y_t     = dinamico_tilde(y);
            [grad2, H2] = Pairs(icp).body2.get_gradH(y);

            dphi2_dq    = grad2.' * [y_t, -1/alpha*eye(3)] * J12;
            d2phi2_dxdq = (1/alpha) * R12 * ...
                          [H2*y_t - dinamico_tilde(grad2), -1/alpha*H2] * J12;
            d2phi2_dadq = -(1/alpha) * (grad2 + H2*y).' * ...
                          [y_t, -1/alpha*eye(3)] * J12;

            dF_dq = [ zeros(1,6);
                      dphi2_dq;
                      lambda2 * d2phi2_dxdq;
                      lambda2 * d2phi2_dadq ];

            dz_dq     = -out.J \ dF_dq;
            dalpha_dq = dz_dq(4, :);
            dc(row, idx) = (-dalpha_dq).';
        end
    end
end

function [path, ok] = rrt_plan(q_start, q_end, prob, cfg)
% Basic RRT in joint space. Returns path as 6 x M (start to goal).
    nodes  = q_start;          % 6 x K, columns are nodes
    parent = 0;                % parent index per node (0 for root)
    ok     = false;

    for it = 1:cfg.maxIter
        % --- sample ---
        if rand < cfg.goalBias
            q_rand = q_end;
        else
            q_rand = cfg.lb + (cfg.ub - cfg.lb) .* rand(6,1);
        end

        % --- nearest neighbor ---
        d = vecnorm(nodes - q_rand, 2, 1);
        [~, iNear] = min(d);
        q_near = nodes(:, iNear);

        % --- steer ---
        delta = q_rand - q_near;
        nrm = norm(delta);
        if nrm < 1e-9, continue; end
        q_new = q_near + (min(cfg.stepSize, nrm) / nrm) * delta;

        % --- collision check on edge ---
        if ~edge_feasible(q_near, q_new, prob)
            continue;
        end

        % --- add node ---
        nodes(:, end+1) = q_new;     %#ok<AGROW>
        parent(end+1)   = iNear;     %#ok<AGROW>

        % --- goal check ---
        if norm(q_new - q_end) < cfg.goalTol
            if edge_feasible(q_new, q_end, prob)
                nodes(:, end+1) = q_end;       %#ok<AGROW>
                parent(end+1)   = size(nodes,2) - 1;  %#ok<AGROW>
                ok = true;
                break;
            end
        end
    end

    if ~ok
        path = [];
        return;
    end

    % --- backtrack ---
    idx  = size(nodes, 2);
    path = nodes(:, idx);
    while parent(idx) > 0
        idx  = parent(idx);
        path = [nodes(:, idx), path];   %#ok<AGROW>
    end
end

function tf = edge_feasible(qa, qb, prob)
% Check feasibility of straight-line segment qa->qb at sub-step resolution.
    L = norm(qb - qa);
    nSub = max(2, ceil(L / 0.05));   % ~0.05 rad subdivision
    s = linspace(0, 1, nSub);
    tf = true;
    for k = 1:nSub
        q = qa + (qb - qa) * s(k);
        if ~config_feasible(q, prob)
            tf = false;
            return;
        end
    end
end

function tf = config_feasible(q, prob)
% Single-config feasibility check using the same collision primitives as nl_con.
    robot = prob.Linkage;
    Pairs = prob.Pairs;

    % z-floor
    g_ee = getTransform(robot, q, "ee_link");
    if -g_ee(3,4) > 0
        tf = false; return;
    end

    % collision pairs
    for icp = 1:numel(Pairs)
        g1  = getTransform(robot, q, Pairs(icp).body1.bodyName);
        out = Pairs(icp).solveContact(g1, eye(4));
        if (-out.alpha + 1 + prob.eps_clear) > 0
            tf = false; return;
        end
    end
    tf = true;
end

function q_out = resample_path(path_in, N)
% Resample 6 x M path to 6 x N by arc-length-uniform interpolation.
    M = size(path_in, 2);
    if M == 1
        q_out = repmat(path_in, 1, N);
        return;
    end
    seg = vecnorm(diff(path_in, 1, 2), 2, 1);
    s   = [0, cumsum(seg)];
    s_q = linspace(0, s(end), N);
    q_out = zeros(6, N);
    for j = 1:6
        q_out(j, :) = interp1(s, path_in(j, :), s_q, 'linear');
    end
end