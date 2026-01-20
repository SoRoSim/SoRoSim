load("Drone.mat")

beta = 20;
n = 1;
shape_id = 3;
S1.RobotBodies = SorosimContactBody.empty(1,0);
a = 0.15;
b = a;
c = 0.05;

params = idcol_make_params(shape_id, n, a, b, c);

S1.RobotBodies(1) = SorosimContactBody(1, shape_id, params);

S1.EnvironmentBodies = SorosimContactBody.empty(8,0); 

% cylinder
Rb = 1; Rt = 1; a = 3; b = 3; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(1) = SorosimContactBody(1, shape_id, params);
S1.EnvironmentBodies(1).g_JC = [eul2rotm([0,pi/2,0]) [1.25;0;0];[0 0 0 1]];

% Cube
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [1 1 1 1 1 1]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.EnvironmentBodies(2) = SorosimContactBody(2, shape_id, params);
S1.EnvironmentBodies(2).g_JC = [eul2rotm([0,0,0]) [3;2;-2];[0 0 0 1]];

% Cube
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [1 1 1 1 1 1]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.EnvironmentBodies(3) = SorosimContactBody(2, shape_id, params);
S1.EnvironmentBodies(3).g_JC = [eul2rotm([0,0,0]) [3;-2;-2];[0 0 0 1]];

% cuboid
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [0.5 3 1 0.5 3 1]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.EnvironmentBodies(4) = SorosimContactBody(2, shape_id, params);
S1.EnvironmentBodies(4).g_JC = [eul2rotm([0,0,0]) [3;0;0];[0 0 0 1]];

% Frusta
Rb = 0.5; Rt = 1; a = 1; b = 4; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(5) = SorosimContactBody(1, shape_id, params);
S1.EnvironmentBodies(5).g_JC = [eul2rotm([0,-pi/2,0]) [5.0;0;-2];[0 0 0 1]];
S1.EnvironmentBodies(5).mesh_opt.margin = 2;


% cylinder
Rb = 1; Rt = 1; a = 3; b = 3; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(6) = SorosimContactBody(1, shape_id, params);
S1.EnvironmentBodies(6).g_JC = [eul2rotm([0,pi/2,0])*eul2rotm([0.3 0 0]) [7.5;1.5;0];[0 0 0 1]];

% cylinder
Rb = 1; Rt = 1; a = 3; b = 3; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(7) = SorosimContactBody(1, shape_id, params);
S1.EnvironmentBodies(7).g_JC = [eul2rotm([0,pi/2,0])*eul2rotm([-0.3 0 0]) [9;-1.5;0];[0 0 0 1]];

% Cube
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = 2*[1 1 1 1 1 1]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.EnvironmentBodies(8) = SorosimContactBody(2, shape_id, params);
S1.EnvironmentBodies(8).g_JC = [eul2rotm([0,0,0]) [12;0;-1];[0 0 0 1]];

%pairs

for ie = 1:S1.ne
    S1.Pairs(ie) = SorosimContactPair(S1.RobotBodies(1), S1.EnvironmentBodies(ie), ie);
end


%%
S1.Pairs(5) = SorosimContactPair(S1.RobotBodies(1), S1.EnvironmentBodies(5), 5);
S1.Pairs(5).use_warmstart=false;