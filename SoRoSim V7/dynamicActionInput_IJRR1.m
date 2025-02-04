function [action,q_k,qd_k] = dynamicActionInput_IJRR1(t) 
% user can define action as an input of time
% if any joint is controlled by q, action includes its second time derivative and q_k and qd_k for those joints must be provided
% action = [u_k;qdd_k]

u = [-50-50*sin(0.5*pi*t);-80+(-20+80)/10*t;-50-50*sin(pi*t);0;-50];

if t>10/3
    u(5) = 0;
end

if t>10*2/3
    u(4) = -50;
end

action = u;

q_k = [];
qd_k = [];

end