%% Multibody demo (SoRoSim) 
% What you will likely tweak:
%   - Contact stiffness S1.penalty.k_n
%   - dt / tmax
%   - Number of bodies N_bodies and spawn radius R_spawn
%   - Initial speed toward origin
%   - Turn Jacobian/video/save on/off

clear; close all; clc;

%% ------------------------ Setup ------------------------
%make sure startup.m is run

load("Multibody.mat") %loads S1 as linkage

% Contact / penalty params
S1.penalty.k_n = 2e5;

% Simulation knobs
opts.dt          = 0.01;
opts.tmax        = 5;
opts.useJacobian = true;
opts.displayTime = false;
opts.video       = true;
opts.save_data   = false;

% Initial condition knobs
N_bodies = S1.N;    % number of bodies to place
R_spawn  = 6;       % spawn radius (world units)
speed0   = 2.0;     % initial speed magnitude toward origin

%% ------------------------ Initial poses ------------------------
% Generate N_bodies poses roughly uniformly on a sphere of radius R_spawn
G = make_random_poses_on_sphere(N_bodies, R_spawn);

% Optional: randomize ordering (kept from your original)
G = permute_pose_blocks(G);

% Assign to model
S1.g_ini = G;

%% ------------------------ Initial state (q, qd) ------------------------
% Here we assume each body has 6 DOF => state dims = 12*N (q and qd)
N = S1.N;                         % trust the model's N
q0  = zeros(6*N, 1);
qd0 = initial_twists_toward_origin(S1.g_ini, speed0);

x0 = [q0; qd0];

%% ------------------------ ODE definition + options ------------------------
dt   = opts.dt;
tmax = opts.tmax;

ODEFn = @(t,x) derivatives_multibody(S1, t, x, [], [], opts);

if opts.useJacobian
    JFn = @(t,x) ODEJacobian_multibody(S1, t, x, [], []);
    ode_opts = odeset( ...
        'MaxStep', dt, ...
        'RelTol',  1e-3, ...
        'AbsTol',  1e-6, ...
        'Jacobian', JFn );
else
    ode_opts = odeset( ...
        'MaxStep', dt, ...
        'RelTol',  1e-3, ...
        'AbsTol',  1e-6 );
end

tspan = 0:dt:tmax;

%% ------------------------ Simulate ------------------------
tic
[t, qqd] = ode15s(ODEFn, tspan, x0, ode_opts);
toc

%% ------------------------ Visualize / save ------------------------
if opts.video
    plotqt_multibody(S1, t, qqd, 'record', true);
end

if opts.save_data
    save('MultibodyDynamics.mat', 't', 'qqd');
end

%% ========================= Helper functions =========================

function G = make_random_poses_on_sphere(N, R)
% Returns stacked 4x4 transforms: G is (4N)x4.
% Positions are distributed using a Fibonacci sphere; orientations random.
G = zeros(4*N, 4);

phi = (1 + sqrt(5))/2;  % golden ratio for Fibonacci sphere

for i = 1:N
    % Fibonacci sphere point
    t = (i-0.5)/N;
    z = 1 - 2*t;
    r = sqrt(max(0, 1 - z^2));
    ang = 2*pi*i/phi;
    p = R * [r*cos(ang); r*sin(ang); z];

    % Random rotation via random unit quaternion
    q = randn(4,1); q = q / norm(q);
    Rw = quat_to_rotm_wxyz(q);

    g = eye(4);
    g(1:3,1:3) = Rw;
    g(1:3,4)   = p;

    G(4*(i-1)+1:4*i, :) = g;
end
end

function Gperm = permute_pose_blocks(G)
% Randomly permute the 4x4 blocks of G.
N = size(G,1)/4;
perm = randperm(N);
Gperm = zeros(size(G));
for k = 1:N
    src = perm(k);
    Gperm(4*(k-1)+1:4*k, :) = G(4*(src-1)+1:4*src, :);
end
end

function qd0 = initial_twists_toward_origin(G, speed)
% Builds stacked body twists (6N x 1) pointing toward origin in WORLD frame,
% then mapped to body coordinates via Ad_{g^{-1}}.
N = size(G,1)/4;
qd0 = zeros(6*N,1);

for i = 1:N
    gi = G(4*(i-1)+1:4*i, :);
    p  = gi(1:3,4);

    dir = -p;
    nrm = norm(dir);

    if nrm < 1e-12
        v = [0;0;0];
    else
        v = speed * dir / nrm;
    end

    omega = [0;0;0];  % no initial spin

    % Twist for body i (body coordinates)
    qd0(6*(i-1)+1:6*i) = dinamico_Adjoint(ginv(gi)) * [omega; v];
end
end

function R = quat_to_rotm_wxyz(q)
% q = [qw qx qy qz]^T (wxyz convention)
qw = q(1); qx = q(2); qy = q(3); qz = q(4);

R = [1-2*(qy^2+qz^2),   2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
     2*(qx*qy + qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz - qx*qw);
     2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx^2+qy^2)];
end
