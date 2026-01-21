% Translation-only kinematic trajectory optimization using iDCOL constraints
% - RRT initialization (collision-free if possible)
% - fmincon (interior-point) with analytical gradients for objective + constraints

clc; close all;

%% ---------- Load drone linkage ----------
load('Drone.mat');       % expects S1 inside
Linkage = S1;

%% ---------- Problem setup ----------
N  = 60;                     % number of knot points
p0 = [0; 0; 0];              % start position
pT = [13; 0; 2];             % goal position

% Bounds for translation: y,z in [-3,3], x in [0,13]
lbP = [  0; -3; -3];
ubP = [ 13;  3;  3];

% Objective weights
prob.N        = N;
prob.Linkage  = Linkage;
prob.w_smooth = 1.0;
prob.w_length = 0.05;

% Collision clearance parameter in constraint: c = -alpha + 1 + eps_clear
prob.eps_clear  = 5e-2;
prob.dmax_step  = 0.35;      % step constraint (units same as p)

%% ---------- Initial guess: RRT path (preferred), else straight line ----------
rrt.maxIter = 5000;
rrt.step    = 0.30;          % keep <= prob.dmax_step to avoid immediate infeasibility
rrt.goalTol = 0.50;
rrt.pGoal   = 0.10;

t_rrt = tic;
[pathPts, ok] = rrt3d_plan(p0, pT, lbP, ubP, Linkage, prob.eps_clear, rrt);
t_rrt = toc(t_rrt);

t_resamp = 0;
if ok
    t_resamp = tic;
    P0 = resample_polyline(pathPts, N);
    t_resamp = toc(t_resamp);
else
    warning('RRT failed, using straight-line initialization (may be infeasible).');
    s  = linspace(0,1,N);
    P0 = (1-s).*p0 + s.*pT;
end

fprintf('RRT time: %.3f s | Resample time: %.3f s | ok=%d\n', t_rrt, t_resamp, ok);


x0 = P0(:);

%% ---------- Bounds (and hard-fix endpoints via bounds) ----------
lb = repmat(lbP, N, 1);
ub = repmat(ubP, N, 1);

idx_start = 1:3;
idx_goal  = (3*(N-1)+1):(3*N);
lb(idx_start) = p0;  ub(idx_start) = p0;
lb(idx_goal)  = pT;  ub(idx_goal)  = pT;

%% ---------- fmincon options ----------

alg = 'interior-point';   % primary attempt
dispMode = 'iter';  % 'iter' or 'final' or 'off'
useGrads = true;    % true: analytical gradients, false: finite-difference baseline

opts = optimoptions('fmincon', ...
    'Algorithm', alg, ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 2e5, ...
    'MaxIterations', 1000, ...
    'SpecifyObjectiveGradient', useGrads, ...
    'SpecifyConstraintGradient', useGrads, ...
    'EnableFeasibilityMode', false, ...
    'ConstraintTolerance', 1e-6, ...
    'OptimalityTolerance', 1e-4, ...
    'ScaleProblem', true,...
    'Display', dispMode);

%% ---------- Solve (robust, user-friendly logging) ----------
solveCfg.trySQPOnWarning = true;     % fallback only if interior-point gets ill-conditioned
solveCfg.quietSingularWarnings = true; % suppress MATLAB spam only for these warnings
solveCfg.feasMarginOK = 1e-4;        % accept if max(c) <= -feasMarginOK

t0 = tic;
sol = solve_kinematic_fmincon(prob, x0, lb, ub, opts, solveCfg, useGrads);
t_opt = toc(t0);

fprintf('Optimization time: %.3f s | alg=%s | status=%s | iters=%d | firstorderopt=%.3e | max(c)=%.3e | singularWarn=%d\n', ...
    t_opt, sol.alg, sol.status, sol.out.iterations, sol.out.firstorderopt, sol.max_c, sol.singularWarn);

xsol = sol.x;


%% ---------- Plot ----------
P = reshape(xsol, 3, N);

plotq_drone(Linkage);
hold on; grid on;

h_rrt = plot3(P0(1,:), P0(2,:), P0(3,:), '-ob', 'LineWidth', 1.5);
h_opt = plot3(P(1,:),  P(2,:),  P(3,:),  '-or', 'LineWidth', 1.5);

plot3(p0(1), p0(2), p0(3), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
plot3(pT(1), pT(2), pT(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

xlabel('x'); ylabel('y'); zlabel('z');
legend([h_rrt, h_opt], {'RRT initial path', 'Optimized path'}, 'Location', 'best');

%% ---------- Save ----------
P_rrt = P0;
P_opt = P;
save('OptimalPath.mat','P_rrt','P_opt');   % use MAT, not M, unless you truly need an .m script

%% plot video
plotvideo = true;
if plotvideo
    plotqt_drone(Linkage, P, 10, 'traj.mp4'); %10 is frame rate
end

%% Helpers

function sol = solve_kinematic_fmincon(prob, x0, lb, ub, opts, cfg, useGrads)
% Robust solve wrapper:
% - catches near-singular warnings and reports them cleanly
% - uses bestfeasible when available
% - validates feasibility and optionally retries with SQP

    if nargin < 6 || isempty(cfg)
        cfg = struct();
    end
    if ~isfield(cfg,'trySQPOnWarning'),         cfg.trySQPOnWarning = true; end
    if ~isfield(cfg,'quietSingularWarnings'),  cfg.quietSingularWarnings = true; end
    if ~isfield(cfg,'feasMarginOK'),           cfg.feasMarginOK = 1e-4; end

    % --- run primary solve (as configured in opts) ---
    [x_end, fval, exitflag, out, singularWarn] = run_once(prob, x0, lb, ub, opts, cfg.quietSingularWarnings, useGrads);

    x = pick_bestfeasible(x_end, out);
    if useGrads
        [c_best, ~] = nl_con(x, prob);
    else
        [c_best, ~] = nl_con_nojac(x, prob);
    end
    max_c = max(c_best);

    sol = pack_solution(x, fval, exitflag, out, opts.Algorithm, "primary", max_c, singularWarn);

    % Accept if feasible with margin
    if max_c <= -cfg.feasMarginOK
        sol.status = "ok";
        return;
    end

    % Optionally retry with SQP if warning happened or margin is weak
    if cfg.trySQPOnWarning && (singularWarn || max_c > -cfg.feasMarginOK)
        fprintf('[solve] Switching to SQP fallback (singularWarn=%d, max(c)=%.3e).\n',singularWarn, max_c);
        opts2 = optimoptions(opts, 'Algorithm','sqp', 'Display','final');
        [x_end2, fval2, exitflag2, out2, singularWarn2] = run_once(prob, x, lb, ub, opts2, cfg.quietSingularWarnings, useGrads);

        x2 = pick_bestfeasible(x_end2, out2);
        if useGrads
            [c2, ~] = nl_con(x2, prob);
        else
            [c2, ~] = nl_con_nojac(x2, prob);
        end

        max_c2 = max(c2);

        sol2 = pack_solution(x2, fval2, exitflag2, out2, 'sqp', "fallback_sqp", max_c2, singularWarn2);

        % Choose better feasibility margin first, then objective
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

function [x_end, fval, exitflag, out, singularWarn] = run_once(prob, x0, lb, ub, opts, quietSingularWarnings, useGrads)

    singularWarn = false;

    ids = {'MATLAB:nearlySingularMatrix','MATLAB:singularMatrix','MATLAB:illConditionedMatrix'};
    old = cell(size(ids));
    for i = 1:numel(ids)
        old{i} = warning('query', ids{i});
        if quietSingularWarnings
            warning('off', ids{i});
        else
            warning('on', ids{i});
        end
    end

    lastwarn('');

    if useGrads
        obj = @(x) obj_fun(x, prob);
        con = @(x) nl_con(x, prob);
    else
        obj = @(x) obj_fun_nojac(x, prob);
        con = @(x) nl_con_nojac(x, prob);
    end

    [x_end, fval, exitflag, out] = fmincon(obj, x0, [],[],[],[], lb, ub, con, opts);

    [~, wid] = lastwarn;
    if any(strcmp(wid, ids))
        singularWarn = true;
    end

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
