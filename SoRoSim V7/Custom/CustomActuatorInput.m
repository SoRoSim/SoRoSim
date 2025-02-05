%Function to calculate action as a function of state and time
%Last modified 05/02/2025

function [action,daction_dx] = CustomActuatorInput(Linkage,x,t)

% x is a vector of unknowns for statics x = [q_u;u_u;lambda], x is [q, qdot] for dynamics
% t is time
% action = [u_k;q_k] for statics and [u_k;qdd_k] for dynamics

action = zeros(Linkage.nact,1);
nx = length(x);

daction_dx = zeros(Linkage.nact,nx);


end

