function [action,q_k,qd_k] = dynamicActionInputSM(t) 
% user can define action as an input of time
% if any joint is controlled by q, action includes its second time derivative and q_k and qd_k for those joints must be provided
% action = [u_k;qdd_k]
    u_k = zeros(6,1);

    u_k(1) = -5*t;
    u_k(2) = -20*sin(t)^2;
    u_k(3) = -1*t^2;
    u_k(4) = -10*t;
    u_k(5) = -20*cos(t)^2;
    u_k(6) = -5*t;
    
    action = u_k;
    q_k  = [];
    qd_k = [];

end