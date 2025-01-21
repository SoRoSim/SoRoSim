% Plotting the rope at q
function [constraint_surface, root1, root2] = find_constraint(S1,qu_uq_l, constraint_height)
    figure
    S1.plotq(qu_uq_l(1:S1.ndof));
    
    g = S1.FwdKinematics(qu_uq_l(1:S1.ndof));
    x1 = g(5:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    y1 = g(6:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    z1 = g(7:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    
    
    
    x2 = g(4*(length(S1.CVTwists{1}(2).Xs)+4)+1:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4);
    y2 = g(4*(length(S1.CVTwists{1}(2).Xs)+4)+2:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4);
    z2 = g(4*(length(S1.CVTwists{1}(2).Xs)+4)+3:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4);
    
    
    xbars = linspace(0,1);
    X1 = polyfit(S1.CVTwists{1}(2).Xs, x1,3);
    Y1 = polyfit(S1.CVTwists{1}(2).Xs, y1, 3);
    Z1 = polyfit(S1.CVTwists{1}(2).Xs, z1,3);
    
    X2 = polyfit(S1.CVTwists{3}(2).Xs, x2,3);
    Y2 = polyfit(S1.CVTwists{3}(2).Xs, y2, 3);
    Z2 = polyfit(S1.CVTwists{3}(2).Xs, z2,3);
    
    
%     plot3(polyval(X1,xbars), polyval(Y1,xbars), polyval(Z1,xbars));
%     hold on
%     plot3(polyval(X2,xbars), polyval(Y2,xbars), polyval(Z2,xbars));
    
    % Finding the position of the holes 
    constraint_surface.height = constraint_height;
    Z_temp = Z1;
    Z_temp(end) = Z_temp(end)-constraint_surface.height;
    roots1 = roots(Z_temp);
    idx = find(roots1>=0 & roots1<=1);
    root1 = roots1(idx);
    constraint_surface.hole_1 = [polyval(X1,roots1(idx)), polyval(Y1,roots1(idx))];
    Z_temp = Z2;
    Z_temp(end) = Z_temp(end)-constraint_surface.height;
    roots1 = roots(Z_temp);
    idx = find(roots1>=0 & roots1<=1);
    root2 = roots1(idx);
    constraint_surface.hole_2 = [polyval(X2,roots1(idx)), polyval(Y2,roots1(idx))];
    constraint_surface.radius = 0.05;
    
    theta = linspace(0,2*pi);
    xh1 = constraint_surface.radius*cos(theta) +constraint_surface.hole_1(1);
    yh1 = constraint_surface.radius*sin(theta) + constraint_surface.hole_1(2);
    zh1 = ones(1,length(theta))*constraint_surface.height;
    
    xh2 = constraint_surface.radius*cos(theta) + constraint_surface.hole_2(1);
    yh2 = constraint_surface.radius*sin(theta) + constraint_surface.hole_2(2);
    zh2 = ones(1,length(theta))*constraint_surface.height;
    
    plot3(xh1, yh1, zh1,'r')
    hold on
    plot3(xh2, yh2, zh2,'r');
end
%%
% g_des_final = [0.0000         0   -1.0000    0.3
%                 0    1.0000         0         0
%                 1.0000         0    0.0000    0.438
%                 0         0         0    1.0000];
% 
% g_platform_0 = [0.0000         0   -1.0000    0.3000
%                 0    1.0000         0         0
%                 1.0000         0    0.0000    0.4380
%                 0         0         0    1.0000];
% n_points = 10;
% qu_uq_l0 = [qu_uq_l ones(10,1)*root1 ones(10,1)*root2];
% 
% [c , ceq] = constraints(S1, qu_uq_l0, n_points, constraint_surface,g_des_final, g_platform_0)
% 
% function [c, ceq] = constraints(S1, qu_uq_l,n_points,constrain_surface, g_desired, g_0)
%     c = [];
%     qu_uq_l = reshape(qu_uq_l, n_points, S1.ndof + 20);
%     qul = [qu_uq_l(:,1:S1.ndof)'; qu_uq_l(:,S1.ndof+13:S1.ndof+18)']';
%     uq = qu_uq_l(:,S1.ndof+1:S1.ndof+12);
%     x1 = qu_uq_l(:,S1.ndof+19);
%     x2 = qu_uq_l(:,S1.ndof+20);
%     ceq = [];
%     for i =1:n_points
%         g = S1.FwdKinematics(qu_uq_l(i,1:S1.ndof));
%         X1s = S1.CVTwists{1}(2).Xs;
%         X2s = S1.CVTwists{3}(2).Xs;
%         X1 = polyfit(X1s, g(5:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
%         Y1 = polyfit(X1s, g(6:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
%         Z1 = polyfit(X1s, g(7:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
%         xh1 = [polyval(X1,x1(i)) polyval(Y1,x1(i)) polyval(Z1,x1(i))];
%         eq1 = norm([constrain_surface.hole_1 constrain_surface.height] - xh1);
%         X2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+1:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
%         Y2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+2:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
%         Z2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+3:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
%         xh2 = [polyval(X2,x2(i)) polyval(Y2,x2(i)) polyval(Z2,x2(i))];
%         eq2 = norm([constrain_surface.hole_2 constrain_surface.height] - xh2);
%         lsqoptions = optimoptions('lsqlin','Display','off');
%         magnifier = 1;
%         eq3 = Equilibrium_optim(S1,qul(i,:)',uq(i,:)', magnifier, lsqoptions);
%         ceq = [ceq; eq3];%[ceq;eq1; eq2; eq3];
%         c = [c;eq1-0.05;eq2-0.05];
%     end
%     qT = qu_uq_l(end,1:S1.ndof);
%     gs = S1.FwdKinematics(qT);
%     g_platformT = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
%     eq_4 = piecewise_logmap(ginv(g_platformT)*g_desired);
%     q0 = qu_uq_l(1,1:S1.ndof);
%     gs = S1.FwdKinematics(q0);
%     g_platform0 = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
%     eq_5 = piecewise_logmap(ginv(g_platform0)*g_0);
%     ceq = [ceq; eq_4;eq_5];
% end