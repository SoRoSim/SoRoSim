load("RRT_initial_seed.mat")
g_init = Linkage.FwdKinematics(qu_uq_lfull(1,:),8);
g_init_left = g_init(5:8,:);
g_init_right = Linkage.FwdKinematics(qu_uq_lfull(1,:),19);
g_init_right = g_init_right(5:8,:);
g_init = [g_init_left; g_init_right];
q_a_init = [qu_uq_lfull(1,1:7)'; qu_uq_lfull(1,Linkage.ndof-6:Linkage.ndof)'];
% initial_guess_start = [qu_uq_lfull(1,8:Linkage.ndof-7)';qu_uq_lfull(1,Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; qu_uq_lfull(1, Linkage.ndof+Linkage.nact+1:end)'];

fk_handle  = @(q) FK_function(Linkage,q);    % returns 4x4 T
jac_handle = @(q) Jac_function(Linkage,q);   % returns 6 x n

q0 = zeros(7,1);
T_des = g_init(1:4,:);

% optional: posture objective gradient (push towards q_pref)
q_pref = zeros(7,1);
nullObj = @(q) 2*(q - q_pref);   % gradient of ||q-q_pref||^2
qmin = [-1.70167993878; -2.147; -3.05417993878; -0.05; -3.059; -1.57079632679; -3.059];
qmax = [1.70167993878; 1.047; 3.05417993878; 2.618; 3.059; 2.094; 3.059];
[q_sol, info] = soroIK_se3_dls(fk_handle, jac_handle, q0, T_des, ...
    'MaxIter', 400, 'Damping', 1e-2, 'StepLimit', 0.1, ...
    'Qmin', qmin, 'Qmax', qmax, ...
    'NullObjective', nullObj, 'NullGain', 0);

function J = Jac_function(Linkage,q_a)
    q = zeros(Linkage.ndof,1);
    q(1:7) = q_a(1:7);
    J = Linkage.Jacobian(q,8);
    J = J(7:12,1:7);
end
function g= FK_function(Linkage, q_a)
    q= zeros(Linkage.ndof,1);
    q(1:7) = q_a(1:7);
    g= Linkage.FwdKinematics(q,8);
    g = g(5:8,:);
end

function [q, info] = soroIK_se3_dls(fk_func, jac_func, q0, T_des, varargin)
% soroIK_se3_dls  Damped-least-squares IK using SE(3) logmap (twist error).
%
%  [q, info] = soroIK_se3_dls(fk_func, jac_func, q0, T_des, ...)
%
% Inputs:
%   fk_func  - function handle: T_cur = fk_func(q)  (4x4 homogeneous)
%   jac_func - function handle: J = jac_func(q)    (6 x n) with rows [Jv; Jw]
%   q0       - initial joint vector (n x 1)
%   T_des    - desired end-effector pose (4x4)
%
% Name-Value options (defaults):
%   'MaxIter'    - 100
%   'TolPos'     - 1e-4  (m)
%   'TolRot'     - 1e-4  (rad)  (norm of axis-angle)
%   'Damping'    - 1e-2
%   'StepLimit'  - 0.2   (rad max per joint per iter)
%   'Qmin'       - []    (n x 1)
%   'Qmax'       - []    (n x 1)
%   'NullObjective' - [] (handle @(q)->gradH (n x 1))
%   'NullGain'   - 0.1
%   'UsePinv'    - false  (use damped pseudo-inverse style)
%
% Outputs:
%   q    - final joint vector (n x 1)
%   info - struct: success (bool), iterations, pos_err, rot_err, last_dq_norm
%
% Example:
%   fk = @(q) my_fk(q);
%   Jf = @(q) my_jac(q);
%   [qsol, info] = soroIK_se3_dls(fk, Jf, q0, T_des, 'Damping', 1e-2);
%

% ---------------- parse options ----------------
p = inputParser;
addParameter(p,'MaxIter',100);
addParameter(p,'TolPos',1e-4);
addParameter(p,'TolRot',1e-4);
addParameter(p,'Damping',1e-2);
addParameter(p,'StepLimit',0.2);
addParameter(p,'Qmin',[]);
addParameter(p,'Qmax',[]);
addParameter(p,'NullObjective',[]);
addParameter(p,'NullGain',0.1);
addParameter(p,'UsePinv',false);
parse(p,varargin{:});
opts = p.Results;

q = q0(:);
n = numel(q);

info.success = false;
info.iterations = 0;
info.pos_err = NaN;
info.rot_err = NaN;
info.last_dq_norm = NaN;

for k = 1:opts.MaxIter
    % 1) Forward kinematics
    T_cur = fk_func(q);
    if any(isnan(T_cur(:))) || ~all(size(T_cur)==[4,4])
        error('fk_func must return a 4x4 homogeneous transform.');
    end

    % 2) Compute twist error xi = [v; omega] using SE(3) log
    %    xi is such that expm(se3(xi)) = T_err
    T_err = T_des * ginv(T_cur);
    xi = piecewise_logmap(T_err);
    % e = [v; omega];      % 6x1 twist error (linear then angular)
    xi = [xi(4:6); xi(1:3)];
    pos_err = norm(xi(1:3));
    rot_err = norm(xi(4:6));

    info.pos_err = pos_err;
    info.rot_err = rot_err;

    % Convergence check
    if pos_err < opts.TolPos && rot_err < opts.TolRot
        info.success = true;
        info.iterations = k-1;
        return;
    end

    % 3) Jacobian
    J = jac_func(q);   % should be 6 x n with [Jv; Jw]
    if size(J,1) ~= 6 || size(J,2) ~= n
        error('jac_func must return a 6 x n Jacobian (rows: linear; angular).');
    end

    lambda = opts.Damping;

    % 4) Compute task dq (damped least squares)
    if opts.UsePinv
        % J_pinv_damped = J' * inv(J*J' + lambda^2 I6)
        A6 = J*J' + (lambda^2)*eye(6);
        J_pinv = J' / A6;
        dq_task = J_pinv * xi;
    else
        A = J'*J + (lambda^2)*eye(n);
        rhs = J' * xi;
        % Solve - use backslash for stability
        dq_task = A \ rhs;
    end

    % 5) Nullspace objective (optional)
    dq_null = zeros(n,1);
    if ~isempty(opts.NullObjective)
        gradH = opts.NullObjective(q); % gradient of scalar objective (n x 1)
        if numel(gradH) ~= n
            error('NullObjective must return an n-vector gradient.');
        end
        % compute damped pseudo-inverse for nullspace projector
        A6 = J*J' + (lambda^2)*eye(6);
        J_pinv2 = J' / A6;            %  n x 6
        N = eye(n) - J_pinv2 * J;     % nullspace projector approx
        dq_null = -opts.NullGain * (N * gradH);
    end

    dq = dq_task + dq_null;

    % 6) step limiting
    maxAbs = max(abs(dq));
    if maxAbs > opts.StepLimit
        dq = dq * (opts.StepLimit / maxAbs);
    end

    % 7) apply update and joint limits
    q = q + dq;
    if ~isempty(opts.Qmin), q = max(q, opts.Qmin(:)); end
    if ~isempty(opts.Qmax), q = min(q, opts.Qmax(:)); end

    info.iterations = k;
    info.last_dq_norm = maxAbs;
end

% if reached here, not converged
info.success = false;
end
