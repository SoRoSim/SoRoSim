load("RigidBodies.mat")

S1.penalty.k_n = 2e5;

dynamicsOptions.dt = 0.01;
dynamicsOptions.Jacobian = true;
dynamicsOptions.displayTime = false; %display time
dynamicsOptions.save_data = false;
dynamicsOptions.video = true;
dt = dynamicsOptions.dt;
tmax = 5;

% Generate random pose
N = 10; R = 6;
G = zeros(4*N,4);

phi = (1 + sqrt(5))/2;           % golden ratio
for i = 1:N
    t = (i-0.5)/N;
    z = 1 - 2*t;
    r = sqrt(max(0, 1 - z^2));
    ang = 2*pi*i/phi;

    p = R * [r*cos(ang); r*sin(ang); z];

    q = randn(4,1); q = q/norm(q);
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    Rw = [1-2*(qy^2+qz^2),   2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
          2*(qx*qy + qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz - qx*qw);
          2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx^2+qy^2)];

    g = eye(4); g(1:3,1:3)=Rw; g(1:3,4)=p;
    G(4*(i-1)+1:4*i,:) = g;
end

perm = randperm(N);
Gperm = zeros(size(G));

for k = 1:N
    src = perm(k);
    Gperm(4*(k-1)+1:4*k, :) = G(4*(src-1)+1:4*src, :);
end

S1.g_ini = Gperm;
%S1.plotq

N = S1.N;
q0 = zeros(6*N,1);
qd0 = zeros(6*N,1);

speed = 2.0;   % m/s equivalent scale (tune)

for i = 1:N
    gi = S1.g_ini(4*(i-1)+1:4*i, :);
    p  = gi(1:3,4);

    dir = -p;
    nrm = norm(dir);
    if nrm < 1e-12
        v = [0;0;0];
    else
        v = speed * dir / nrm;
    end

    omega = [0;0;0];      % no spin initially

    qd0(6*(i-1)+1:6*i) = dinamico_Adjoint(ginv(gi)) * [omega; v];  % twist for body i
end

x0 = [q0;qd0];   % 60x1

if dynamicsOptions.Jacobian
    options = odeset('OutputFcn',@(t,y,flag) odeprogress(t,y,flag,S1,dynamicsOptions.plotProgress),'MaxStep', dt,'RelTol',1e-3,'AbsTol',1e-6,'Jacobian',@(t,x) ODEJacobian2(S1,t,x,[],[]));% CHANGE NAMES
else
    options = odeset('OutputFcn',@(t,y,flag) odeprogress(t,y,flag,S1,dynamicsOptions.plotProgress),'MaxStep', dt,'RelTol',1e-3,'AbsTol',1e-6);% No Jacobian
end
ODEFn = @(t,x) derivatives2(S1,t,x,[],[],dynamicsOptions); %ODE function which computes [qd;qdd]

% profile on
tic
[t,qqd] = ode15s(ODEFn,0:dt:tmax,x0,options); %23
toc
% profile off
% profile viewer

if dynamicsOptions.video
    plotqt_temp2(S1,t,qqd,'record',true)
end
if dynamicsOptions.save_data
    save('Multibody.mat','t','qqd')
end