load("SoftManipulator.mat")
S1.penalty.k_n = 5e4;
% profile on
[t,qqd] = S1.dynamics(zeros(S1.ndof*2,1),@(t) deal([-8*t; -8*t; 0], [], []),'video',false,'Jacobian',true,'t_end',5,'displayTime',true);
% profile off
% profile viewer
plotqt_temp(S1,t,qqd,'record',false);