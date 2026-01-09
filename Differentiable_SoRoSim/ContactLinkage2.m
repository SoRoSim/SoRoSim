load("RigidBodies.mat")

beta = 20;
S1.RobotBodies = SorosimContactBody.empty(S1.N,0);

% cone
Rb = 1; Rt = 1e-3; a = 0.5; b = 1.5; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.RobotBodies(1) = SorosimContactBody(1, shape_id, params);
S1.RobotBodies(1).i_sig = 2;

% Cube
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [1 1 1 1 1 1]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.RobotBodies(2) = SorosimContactBody(2, shape_id, params);
S1.RobotBodies(2).i_sig = 4;

% cuboid
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [1.5 1 0.5 1.5 1 0.5]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.RobotBodies(3) = SorosimContactBody(3, shape_id, params);
S1.RobotBodies(3).i_sig = 6;

% cylinder
Rb = 0.15; Rt = 0.15; a = 1.5; b = 1.5; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.RobotBodies(4) = SorosimContactBody(4, shape_id, params);
S1.RobotBodies(4).i_sig = 8;

% ellipsoid
a = 1.5; b = 1; c = 0.5; n = 1; shape_id = 3;
params = idcol_make_params(shape_id, n, a, b, c);
S1.RobotBodies(5) = SorosimContactBody(5, shape_id, params);
S1.RobotBodies(5).i_sig = 10;

% frusta
Rb = 1; Rt = 0.5; a = 0.7857; b = 2-0.7857; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.RobotBodies(6) = SorosimContactBody(6, shape_id, params);
S1.RobotBodies(6).i_sig = 12;

% polytope
A = 1/sqrt(3)*[ 1   1    1;   
                1  -1   -1;   
               -1   1   -1;    
               -1  -1    1;
               -1  -1   -1;   
               -1   1    1;   
                1  -1    1;    
                1   1   -1];

b = [1; 1; 1; 1; 5/3; 5/3; 5/3; 5/3];
shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.RobotBodies(7) = SorosimContactBody(7, shape_id, params);
S1.RobotBodies(7).i_sig = 14;

% pyramid
A = [  1   4   0;   % x + 4y <= 1.5
       1  -4   0;   % x - 4y <= 1.5
       1   0   4;   % x + 4z <= 1.5
       1   0  -4;   % x - 4z <= 1.5
      -1   0   0];  % -x <= 0.5   (i.e., x >= -0.5)

b = [ 1.5; 1.5; 1.5; 1.5; 0.5];

shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.RobotBodies(8) = SorosimContactBody(8, shape_id, params);
S1.RobotBodies(8).i_sig = 16;

% sphere
R = 1; shape_id = 1;
params = idcol_make_params(shape_id, R);
S1.RobotBodies(9) = SorosimContactBody(9, shape_id, params);
S1.RobotBodies(9).i_sig = 18;

% tetarahedra
A = 1/sqrt(3)*[ 1   1    1;   
                1  -1   -1;   
               -1   1   -1;    
               -1  -1    1];

b = [1; 1; 1; 1];
shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.RobotBodies(10) = SorosimContactBody(10, shape_id, params);
S1.RobotBodies(10).i_sig = 20;


S1.EnvironmentBodies = SorosimContactBody.empty(1,0); %1 plane (lets make it a box)
% NOT working! need to fix later
% %plane (a box) only top part used. make it better later
% A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [20 20 20 20 20 20]'; shape_id = 2;
% params = idcol_make_params(shape_id, beta, A, b);
% S1.EnvironmentBodies(1) = SorosimContactBody(11, shape_id, params);
% S1.EnvironmentBodies(1).g_JC = [eye(3) [0;0;-20];[0 0 0 1]];


% %pairs: plane-body
% for ib = 1:S1.nb
%     S1.Pairs(ib) = SorosimContactPair(S1.RobotBodies(ib), S1.EnvironmentBodies(1), ib);
% end

index = 1;
%pairs: body-body
S1.Pairs = SorosimContactPair.empty(S1.nb*(S1.nb-1)/2,0);
for ib = 1:S1.nb
    for jb = ib+1:S1.nb
        S1.Pairs(index) = SorosimContactPair(S1.RobotBodies(ib), S1.RobotBodies(jb), index);
        index = index+1;
    end
end
