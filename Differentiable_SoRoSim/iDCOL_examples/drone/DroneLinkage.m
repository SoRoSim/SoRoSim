%% Drone: build 1 robot body + multiple environment bodies + pairs
% Edit points:
%   - beta (smoothing) for env convex primitives
%   - drone ellipsoid size (a,b,c)
%   - environment list (shape/pose)
%
% Notes:
%   - Face/vertex data for visualization are generated on first plot.
%     Run once, plot once, then save again to cache mesh data for faster plotting.

clear; clc;
load("Drone.mat") %loads S1 as linkage

%% ------------------------ User knobs ------------------------
beta = 20;

% Drone body (ellipsoid)
n = 1;
shape_id = 4;
a = 0.15; b = 0.15; c = 0.05;

%% ------------------------ RobotBodies (1 body) ------------------------
S1.RobotBodies = SorosimContactBody.empty(1,0);

params = idcol_make_params(shape_id, n, a, b, c);
S1.RobotBodies(1) = SorosimContactBody(1, shape_id, params);

%% ------------------------ EnvironmentBodies ------------------------
S1.EnvironmentBodies = SorosimContactBody.empty(8,0);

% Helper shorthands (no external files)
I4 = [0 0 0 1];
T  = @(R,p) [R p(:); I4];

% Reusable halfspace template for axis-aligned boxes
A_box = [ eye(3); -eye(3) ];

% (1) Cylinder (long)
shape_id = 3;
Rb=0.75; Rt=0.75; a=3; b=3;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(1) = SorosimContactBody(1, shape_id, params);
S1.EnvironmentBodies(1).g_JC = T(eul2rotm([0, pi/2, 0]), [1.25; 0; 0]);

% (2) Cube
shape_id = 2;
bA = [1 1 1 1 1 1]';
params = idcol_make_params(shape_id, beta, A_box, bA);
S1.EnvironmentBodies(2) = SorosimContactBody(2, shape_id, params);
S1.EnvironmentBodies(2).g_JC = T(eul2rotm([0,0,0]), [3; 2; -2]);

% (3) Cube
params = idcol_make_params(shape_id, beta, A_box, bA);
S1.EnvironmentBodies(3) = SorosimContactBody(3, shape_id, params);
S1.EnvironmentBodies(3).g_JC = T(eul2rotm([0,0,0]), [3; -2; -2]);

% (4) Cuboid
bA = [0.5 3 1 0.5 3 1]';
params = idcol_make_params(shape_id, beta, A_box, bA);
S1.EnvironmentBodies(4) = SorosimContactBody(4, shape_id, params);
S1.EnvironmentBodies(4).g_JC = T(eul2rotm([0,0,0]), [3; 0; 0]);

% (5) Frustum
shape_id = 3;
Rb=0.5; Rt=1; a=1; b=4;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(5) = SorosimContactBody(5, shape_id, params);
S1.EnvironmentBodies(5).g_JC = T(eul2rotm([0, -pi/2, 0]), [5.0; 0; -2]);
S1.EnvironmentBodies(5).mesh_opt.margin = 2;

% (6) Cylinder (roty(-90)*rotz(-15))
shape_id = 3;
Rb=1; Rt=1; a=3; b=3;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(6) = SorosimContactBody(6, shape_id, params);
S1.EnvironmentBodies(6).g_JC = [roty(-90)*rotz(-15) [9.5;-1;0]; I4];

% (7) Cylinder (roty(-90)*rotz(15))
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(7) = SorosimContactBody(7, shape_id, params);
S1.EnvironmentBodies(7).g_JC = [roty(-90)*rotz(15) [9.5;-1;0]; I4];

% (8) Big cube
shape_id = 2;
bA = 2*[1 1 1 1 1 1]';
params = idcol_make_params(shape_id, beta, A_box, bA);
S1.EnvironmentBodies(8) = SorosimContactBody(8, shape_id, params);
S1.EnvironmentBodies(8).g_JC = T(eul2rotm([0,0,0]), [12; 0; -1]);


%% ------------------------ Pairs: drone vs each environment body ------------------------
S1.Pairs = SorosimContactPair.empty(S1.ne,0);
for ie = 1:S1.ne
    S1.Pairs(ie) = SorosimContactPair(S1.RobotBodies(1), S1.EnvironmentBodies(ie), ie);
    S1.Pairs(ie).use_warmstart = false;
end

save("Drone.mat")
% Note:
% Face/vertex data are generated lazily on the first plot.
% Run once to generate face/vertex data via plotting, then save again to cache it.
