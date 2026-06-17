clc
clear variables

%contact implicit solver

dt = 0.01;
t_final = 5;
nt = t_final/dt+1;

% Load model
load("SoftManipulator.mat") % loads S1 as Linkage

% Penalty contact parameters
S1.penalty.k_n = 5e4;

% Actuation profile
u = @(t) [-12*t; -8*t; 0];

% Initial state
q0  = zeros(S1.ndof,1);
qd0 = zeros(S1.ndof,1);
q   = zeros(S1.ndof,nt);
qd  = zeros(S1.ndof,nt);
q(:,1)  = q0;
qd(:,1) = qd0;

% Preallocate contact info struct
ndof = S1.ndof;
ncp  = S1.ncp;
contactInfo.activePairs = false(ncp, 1);
contactInfo.x           = zeros(3, ncp);
contactInfo.n           = zeros(3, ncp);
contactInfo.delta       = zeros(1, ncp);
contactInfo.dx_dq       = zeros(3, ndof, ncp);
contactInfo.dn_dq       = zeros(3, ndof, ncp);
contactInfo.ddelta_dq   = zeros(1, ndof, ncp);

% Lagged ICF: gamma_n0 from previous step
gamma_n0 = zeros(ncp, 1);

%% Time integration
% profile on
tic
for i=2:nt
    % i
    t_next = (i-1)*dt;  % t_{n+1}
    
    %step 1
    qd_free    = iEulerStep(S1, q(:,i-1), qd(:,i-1), dt, u(t_next));

    %step 2, can be parallized for each pair
    [~,~,~,~,M,~,~,~,g,J] = dRNEA("getlast"); %M via CBA, g,J from RNEA("")
    [contactInfo.activePairs, contactInfo.delta, contactInfo.x, contactInfo.n, ...
     contactInfo.ddelta_dq, contactInfo.dx_dq, contactInfo.dn_dq] = ...
        collisionDetection(S1, g, J);

    %step 3
    if any(contactInfo.activePairs)
        [qd(:,i), gamma_n0] = contactResolution(S1, M, g, J, qd_free, contactInfo, gamma_n0, dt);
    else
        qd(:,i) = qd_free;
        gamma_n0 = zeros(ncp, 1);  % reset when no contact
    end
    q(:,i)     = q(:,i-1) + dt*qd(:,i);

end
toc
% profile off
% profile viewer

t = 0:dt:5;
% plotqt_softmanipulator(S1, t, q', 'record', true);