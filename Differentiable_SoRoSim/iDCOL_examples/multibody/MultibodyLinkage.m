%% Build RobotBodies (mixed shapes) + all body-body pairs (no helper files)
% Edit points:
%   - beta
%   - each shape block (params)
%   - i_sig indices
%   - number of bodies nb

clear; clc;
load("Multibody.mat") % loads S1 as Linkage and associated links: 'Cone','Cube','Cuboid','Cylinder','Ellipsoid','Frusta','Polytope','Pyramid','Sphere','Tetrahedra'

beta = 20;

%% ---------------- Robot bodies ----------------
nb = S1.N;  % number of robot bodies you define below
S1.RobotBodies = SorosimContactBody.empty(nb,0);

% 1) Cone (frustum with Rt ~ 0)
shape_id = 5;
Rb = 1; Rt = 1e-3; a = 0.5; b = 1.5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.RobotBodies(1) = SorosimContactBody(1, shape_id, params);
S1.RobotBodies(1).i_sig = 2;

% 2) Cube
shape_id = 2;
A = [ eye(3); -eye(3) ];
bA = [1 1 1 1 1 1]';
params = idcol_make_params(shape_id, beta, A, bA);
S1.RobotBodies(2) = SorosimContactBody(2, shape_id, params);
S1.RobotBodies(2).i_sig = 4;

% 3) Cuboid
shape_id = 2;
A = [ eye(3); -eye(3) ];
bA = [1.5 1 0.5 1.5 1 0.5]';
params = idcol_make_params(shape_id, beta, A, bA);
S1.RobotBodies(3) = SorosimContactBody(3, shape_id, params);
S1.RobotBodies(3).i_sig = 6;

% 4) Cylinder (special case of frustum)
shape_id = 5;
Rb = 0.15; Rt = 0.15; a = 1.5; b = 1.5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.RobotBodies(4) = SorosimContactBody(4, shape_id, params);
S1.RobotBodies(4).i_sig = 8;

% 5) Ellipsoid
shape_id = 3;
a = 1.5; b = 1; c = 0.5; n = 1;
params = idcol_make_params(shape_id, n, a, b, c);
S1.RobotBodies(5) = SorosimContactBody(5, shape_id, params);
S1.RobotBodies(5).i_sig = 10;

% 6) Frustum
shape_id = 5;
Rb = 1; Rt = 0.5; a = 0.7857; b = 2-0.7857;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.RobotBodies(6) = SorosimContactBody(6, shape_id, params);
S1.RobotBodies(6).i_sig = 12;

% 7) Polytope (8 facets)
shape_id = 2;
A = 1/sqrt(3)*[ 1   1    1;
                1  -1   -1;
               -1   1   -1;
               -1  -1    1;
               -1  -1   -1;
               -1   1    1;
                1  -1    1;
                1   1   -1 ];
bA = [1; 1; 1; 1; 5/3; 5/3; 5/3; 5/3];
params = idcol_make_params(shape_id, beta, A, bA);
S1.RobotBodies(7) = SorosimContactBody(7, shape_id, params);
S1.RobotBodies(7).i_sig = 14;

% 8) Pyramid (halfspace form)
shape_id = 2;
A = [  1   4   0;
       1  -4   0;
       1   0   4;
       1   0  -4;
      -1   0   0 ];
bA = [ 1.5; 1.5; 1.5; 1.5; 0.5 ];
params = idcol_make_params(shape_id, beta, A, bA);
S1.RobotBodies(8) = SorosimContactBody(8, shape_id, params);
S1.RobotBodies(8).i_sig = 16;

% 9) Sphere
shape_id = 1;
R = 1;
params = idcol_make_params(shape_id, R);
S1.RobotBodies(9) = SorosimContactBody(9, shape_id, params);
S1.RobotBodies(9).i_sig = 18;

% 10) Tetrahedron
shape_id = 2;
A = 1/sqrt(3)*[ 1   1    1;
                1  -1   -1;
               -1   1   -1;
               -1  -1    1 ];
bA = [1; 1; 1; 1];
params = idcol_make_params(shape_id, beta, A, bA);
S1.RobotBodies(10) = SorosimContactBody(10, shape_id, params);
S1.RobotBodies(10).i_sig = 20;

%% ---------------- Environment bodies (empty for now) ----------------
S1.EnvironmentBodies = SorosimContactBody.empty(1,0);

%% ---------------- Build all body-body pairs ----------------

S1.Pairs = SorosimContactPair.empty(nb*(nb-1)/2,0);
idx = 1;
for ib = 1:nb
    for jb = ib+1:nb
        S1.Pairs(idx) = SorosimContactPair(S1.RobotBodies(ib), S1.RobotBodies(jb), idx);
        idx = idx + 1;
    end
end

% Update link inertia if needed:
% syntax: S1.VLinks(i).M = M_new;    i = 1:S1.N

fprintf('Built %d robot bodies and %d body-body pairs.\n', nb, numel(S1.Pairs));
save('Multibody.mat', ...
     'S1','Cone','Cube','Cuboid','Cylinder','Ellipsoid', ...
     'Frusta','Polytope','Pyramid','Sphere','Tetrahedra');
% Note:
% Face/vertex data are generated lazily on the first plot.
% Run once to generate face/vertex data via plotting, then save again to cache it.


