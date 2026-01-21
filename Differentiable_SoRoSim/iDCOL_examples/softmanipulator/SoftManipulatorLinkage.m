%% SoftManipulator: build contact bodies along rod + environment bodies + pairs
% Edit points:
%   - beta (smoothing)
%   - environment shapes + poses (g_JC)

clear; clc;
load("SoftManipulator.mat") % loads S1 as Linkage and associated link L1

beta = 20;

%% ----------------------- Pull rod data -----------------------
% Which CVRod segment to discretize into contact frusta
link_id  = 1;
div_id   = 1;

r_fn = S1.VLinks(link_id).r{1};          % radius function r(s)
Xs   = S1.CVRods{link_id}(div_id+1).Xs;  % normalized arc-length grid in [0,1]
nip  = S1.CVRods{link_id}(div_id+1).nip; % number of integration points
L    = S1.VLinks.L;                      % physical length scale (as used in your model)

% We create one contact body per interior point ii = 2..nip-1
n_rod_bodies = nip - 2;

%% ----------------------- RobotBodies: frusta along the rod -----------------------
shape_id_rod = 5;  % frustum/cylinder primitive in your code
S1.RobotBodies = SorosimContactBody.empty(n_rod_bodies,0);

for ii = 2:(nip-1)
    X  = Xs(ii);

    % Local interval [XL, XR] around X (midpoints)
    if ii == 2
        XL = 0;
        XR = (X + Xs(ii+1))/2;
    elseif ii == nip-1
        XL = (X + Xs(ii-1))/2;
        XR = 1;
    else
        XL = (X + Xs(ii-1))/2;
        XR = (X + Xs(ii+1))/2;
    end

    % Half-lengths around the center point in normalized coordinates
    a = X  - XL;
    b = XR - X;

    % Radii at the ends
    Rb = r_fn(XL);
    Rt = r_fn(XR);

    % Build params in physical length units (a*L and b*L)
    params = idcol_make_params(shape_id_rod, beta, Rb, Rt, a*L, b*L);

    id = ii - 1;  % maps ii=2..nip-1 -> id=1..nip-2
    S1.RobotBodies(id) = SorosimContactBody(id, shape_id_rod, params);

    % Map to the correct strain index (your original convention)
    S1.RobotBodies(id).i_sig = 1 + ii;
end

%% ----------------------- EnvironmentBodies -----------------------
% Environment: how many bodies
n_env = 3;  % (1) box "plane", (2) cylinder, (3) block
S1.EnvironmentBodies = SorosimContactBody.empty(n_env,0);

% (1) Box used as a "plane" (top face effectively used)
shape_id = 2;
A = [ eye(3); -eye(3) ];
bA = 0.3 * ones(6,1);
params = idcol_make_params(shape_id, beta, A, bA);

env_id = n_rod_bodies + 1;
S1.EnvironmentBodies(1) = SorosimContactBody(env_id, shape_id, params);
S1.EnvironmentBodies(1).g_JC = [eye(3) [0.3; 0; -0.4]; 0 0 0 1];

% (2) Cylinder
shape_id = 5;
Rb = 0.05; Rt = 0.05; a = 0.15; b = 0.15;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);

env_id = n_rod_bodies + 2;
S1.EnvironmentBodies(2) = SorosimContactBody(env_id, shape_id, params);
S1.EnvironmentBodies(2).g_JC = [roty(-90) [0.3; 0.15; 0.05]; 0 0 0 1];

% (3) Small block
shape_id = 2;
A = [ eye(3); -eye(3) ];
bA = 0.1 * ones(6,1);
params = idcol_make_params(shape_id, beta, A, bA);

env_id = n_rod_bodies + 3;
S1.EnvironmentBodies(3) = SorosimContactBody(env_id, shape_id, params);
S1.EnvironmentBodies(3).g_JC = [rotz(30) [0.55; 0.15; 0.0]; 0 0 0 1];

%% ----------------------- Contact pairs: (rod bodies) x (env bodies) -----------------------
S1.Pairs = SorosimContactPair.empty(n_env*n_rod_bodies,0);

idx = 1;
for ie = 1:n_env
    for ib = 1:n_rod_bodies
        S1.Pairs(idx) = SorosimContactPair(S1.RobotBodies(ib), S1.EnvironmentBodies(ie), idx);
        idx = idx + 1;
    end
end

fprintf('Built %d rod contact bodies, %d environment bodies, %d pairs.\n', ...
    n_rod_bodies, n_env, numel(S1.Pairs));
