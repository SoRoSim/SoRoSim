%% Load model
load("SoftManipulator.mat") % loads S1 as Linkage

%% Penalty contact parameters
S1.penalty.k_n = 5e4;

%% Actuation profile (edit this)
actuation = @(t) [-8*t; -8*t; 0];   % time-varying input force

%% Wrapper required by S1.dynamics (do not modify unless needed)
inputFcn = @(t) deal(actuation(t), [], []);

%% Initial state
x0 = zeros(2*S1.ndof,1);

%% Run simulation
[t, qqd] = S1.dynamics( ...
    x0, ...
    inputFcn, ...
    'video', false, ...
    'Jacobian', true, ...
    't_end', 5, ...
    'displayTime', true );

%% Visualization
plotqt_softmanipulator(S1, t, qqd, 'record', false);