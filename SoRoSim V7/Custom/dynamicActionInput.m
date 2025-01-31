function [action,q_k,qd_k] = dynamicActionInput(t) 
% user can define action as an input of time
% if any joint is controlled by q, action includes its second time derivative and q_k and qd_k for those joints must be provided
% action = [u_k;qdd_k]
    action = zeros(12,1);

    q_k  = zeros(12,1);
    qd_k = zeros(12,1);

    q_k(4) = 0.05*t;
    qd_k(4) = 0.05;
    q_k(5) = 0.05*t;
    qd_k(5) = 0.05;

    q_k(10) = -0.1*t;
    qd_k(10) = -0.1;

    q_k(9) = -0.1*t;
    qd_k(9) = -0.1;

end