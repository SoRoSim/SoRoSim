function [action,q_k,qd_k] = dynamicActionInput_IJRR2(t) 
% user can define action as an input of time
% if any joint is controlled by q, action includes its second time derivative and q_k and qd_k for those joints must be provided
% action = [u_k;qdd_k]

%7 actuated revolute joints, need q, qd, qdd at these joints
f1 = t*(t-5)*(t-7.5)*(t-10)/(2.5*(2.5-5)*(2.5-7.5)*(2.5-10));
f2 = t*(t-2.5)*(t-7.5)*(t-10)/(5*(5-2.5)*(5-7.5)*(5-10));
f3 = t*(t-2.5)*(t-5)*(t-10)/(7.5*(7.5-2.5)*(7.5-5)*(7.5-10));
f4 = t*(t-2.5)*(t-7)*(t-7.5)/(10*(10-2.5)*(10-5)*(10-7.5));

fd1 = - (32*t^3)/1875 + (36*t^2)/125 - (104*t)/75 + 8/5;
fd2 = (16*t^3)/625 - (48*t^2)/125 + (38*t)/25 - 6/5;
fd3 = - (32*t^3)/1875 + (28*t^2)/125 - (56*t)/75 + 8/15;
fd4 = (8*t^3)/1875 - (34*t^2)/625 + (71*t)/375 - 7/50;

fdd1 = (72*t)/125 - (32*t^2)/625 - 104/75;
fdd2 = (48*t^2)/625 - (96*t)/125 + 38/25;
fdd3 = (56*t)/125 - (32*t^2)/625 - 56/75;
fdd4 = (8*t^2)/625 - (68*t)/625 + 71/375;



qjt = zeros(7,3);
j = 1;
qjt(j,1) = pi/3*sin(0.3*pi*t); qjt(j,2) = pi/3*0.3*pi*cos(0.3*pi*t); qjt(j,3) = -pi/3*(0.3*pi)^2*sin(0.3*pi*t); 
j = 2;
qjt(j,1) = -pi/6*f1+0*f2-pi/6*f3-pi/3*f4; qjt(j,2) = -pi/6*fd1+0*fd2-pi/6*fd3-pi/3*fd4; qjt(j,3) = -pi/6*fdd1+0*fdd2-pi/6*fdd3-pi/3*fdd4; 
j = 3;
qjt(j,1) = pi/6*sin(2*pi*t); qjt(j,2) = pi/6*2*pi*cos(2*pi*t); qjt(j,3) = -pi/6*(2*pi)^2*sin(2*pi*t); 
j = 4;
qjt(j,1) = pi/6*f1+pi/4*f2+0*f3+-pi/6*f4; qjt(j,2) = pi/6*fd1+pi/4*fd2+0*fd3+-pi/6*fd4; qjt(j,3) = pi/6*fdd1+pi/4*fdd2+0*fdd3+-pi/6*fdd4; 
qjt(j,:) = -1*qjt(j,:);
j = 5;
qjt(j,1) = 0; qjt(j,2) = 0; qjt(j,3) = 0; 
j = 6;
qjt(j,1) = -pi/3*sin(0.5*pi*t); qjt(j,2) = -pi/3*0.5*pi*cos(0.5*pi*t); qjt(j,3) = pi/3*(0.5*pi)^2*sin(0.5*pi*t); 
j = 7;
qjt(j,1) = -pi/4*f1+pi/6*f2+-pi/6*f3+-pi/4*f4; qjt(j,2) = -pi/4*fd1+pi/6*fd2+-pi/6*fd3+-pi/4*fd4; qjt(j,3) = -pi/4*fdd1+pi/6*fdd2+-pi/6*fdd3+-pi/4*fdd4;

action = qjt(:,3);
q_k = qjt(:,1);
qd_k = qjt(:,2);
end