function [action,q_k,qd_k] = dynamicActionInput_IJRR3(t) 
% user can define action as an input of time
% if any joint is controlled by q, action includes its second time derivative and q_k and qd_k for those joints must be provided
% action = [u_k;qdd_k]

w_max = pi;
stroke = 0.05;
t_w_max = 0.25; %time at which w_max is reached
alpha = w_max/t_w_max;
theta = 1/2*alpha*t^2;

q1 = stroke/2*sin(theta);
qd1 = stroke/2*cos(theta)*alpha*t;
qdd1 = -stroke/2*sin(theta)*(alpha*t)^2+stroke/2*cos(theta)*alpha;

if t>t_w_max
    % alpha = 0; %w = w_max
    theta = 1/2*alpha*t_w_max^2+w_max*(t-t_w_max);

    q1 = stroke/2*sin(theta);
    qd1 = stroke/2*cos(theta)*w_max;
    qdd1 = -stroke/2*sin(theta)*(w_max)^2;
    
end

action = qdd1;
q_k = q1;
qd_k = qd1;
end