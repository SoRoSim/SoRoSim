load("SoftManipulator.mat")
r_fn = S1.VLinks(1).r{1};
Xs = S1.CVRods{1}(2).Xs;
nip = S1.CVRods{1}(2).nip; %change for general
beta = 20;
shape_id = 5;
S1.RobotBodies = SorosimContactBody.empty(nip-2,0);
L = S1.VLinks.L;

for ii=2:nip-1

    X = Xs(ii);
    g_JC = eye(4);

    if ii==2
        XL = 0;
        XR = (X+Xs(ii+1))/2;
    elseif ii==nip-1
        XL = (X+Xs(ii-1))/2;
        XR = 1;
    else
        XL = (X+Xs(ii-1))/2;
        XR = (X+Xs(ii+1))/2;
    end

    a = X-XL;
    b = XR-X;

    Rb = r_fn(XL);
    Rt = r_fn(XR);

    params = idcol_make_params(shape_id, beta, Rb, Rt, a*L, b*L);

    S1.RobotBodies(ii-1) = SorosimContactBody(ii-1, shape_id, params);
    S1.RobotBodies(ii-1).i_sig = 1+ii;
end
S1.EnvironmentBodies = SorosimContactBody.empty(3,0); %1 plane (lets make it a box), 1 cylinder, 1 block

%plane (a box) only top part used. make it better later
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [0.3 0.3 0.3 0.3 0.3 0.3]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.EnvironmentBodies(1) = SorosimContactBody(nip-2+1, shape_id, params);
S1.EnvironmentBodies(1).g_JC = [eye(3) [0.3;0;-0.4];[0 0 0 1]];

%cylinder
Rb = 0.05; Rt = 0.05; a = 0.15; b = 0.15; shape_id = 5;
params = idcol_make_params(shape_id, beta, Rb, Rt, a, b);
S1.EnvironmentBodies(2) = SorosimContactBody(nip-2+2, shape_id, params);
S1.EnvironmentBodies(2).g_JC = [roty(-90) [0.3;0.15;0.05];[0 0 0 1]];

%block
A = [1 0 0;0 1 0;0 0 1;-1 0 0;0 -1 0; 0 0 -1]; b = [0.1 0.1 0.1 0.1 0.1 0.1]'; shape_id = 2;
params = idcol_make_params(shape_id, beta, A, b);
S1.EnvironmentBodies(3) = SorosimContactBody(nip-2+3, shape_id, params);
S1.EnvironmentBodies(3).g_JC = [rotz(30) [0.55;0.15;0.0];[0 0 0 1]];

%pairs
ncp = 3*(nip-2);
for ie = 1:3
    for ib = 1:nip-2
        S1.Pairs((nip-2)*(ie-1)+ib) = SorosimContactPair(S1.RobotBodies(ib), S1.EnvironmentBodies(ie), 3*(ie-1)+ib);
    end
end