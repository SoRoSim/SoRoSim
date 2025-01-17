%Function to calculate actuation input as a function of state and time
% input = [u_k;q_k]
%Last modified 17/01/2025

function [input,dinput_dx] = CustomActuatorStrength(Linkage,x,t)

% x is a vector of unknowns for statics x = [q_u;u_u;lambda], x is [q_u;qd_u;u_u;lambda] for dynamics
% input = [u_k;q_k] is the input to the model (refer to sorosim language)

% g = Linkage.FwdKinematics(q);
% J = Linkage.Jacobian(q);
% Jdot = Linkage.Jacobiandot(q,qd);
% deta_dq
% detadot_dq
% M = Linkage.GeneralizedMassMatrix(q);
% F = Linkage.GeneralizedExternalCoriolisForce(q,qd,t);
% B = Linage.ActuationMatrix(q);

input = zeros(Linkage.nact,1);
nx = length(x);

dinput_dx = zeros(Linkage.nact,nx);


end

