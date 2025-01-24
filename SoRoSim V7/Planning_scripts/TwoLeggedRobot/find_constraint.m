% Plotting the rope at q
function [constraint_surface, root1, root2] = find_constraint(S1,q, constraint_height)
    figure
    S1.plotq(q);
    
    g = S1.FwdKinematics(q);
    x1 = g(5:4:4*(length(S1.CVRods{1}(2).Xs)+1),4);
    y1 = g(6:4:4*(length(S1.CVRods{1}(2).Xs)+1),4);
    z1 = g(7:4:4*(length(S1.CVRods{1}(2).Xs)+1),4);
    
    
    
    x2 = g(4*(length(S1.CVRods{1}(2).Xs)+4)+1:4:4*(length(S1.CVRods{1}(2).Xs)+4+length(S1.CVRods{3}(2).Xs)),4);
    y2 = g(4*(length(S1.CVRods{1}(2).Xs)+4)+2:4:4*(length(S1.CVRods{1}(2).Xs)+4+length(S1.CVRods{3}(2).Xs)),4);
    z2 = g(4*(length(S1.CVRods{1}(2).Xs)+4)+3:4:4*(length(S1.CVRods{1}(2).Xs)+4+length(S1.CVRods{3}(2).Xs)),4);
    
    
    xbars = linspace(0,1);
    X1 = polyfit(S1.CVRods{1}(2).Xs, x1,3);
    Y1 = polyfit(S1.CVRods{1}(2).Xs, y1, 3);
    Z1 = polyfit(S1.CVRods{1}(2).Xs, z1,3);
    
    X2 = polyfit(S1.CVRods{3}(2).Xs, x2,3);
    Y2 = polyfit(S1.CVRods{3}(2).Xs, y2, 3);
    Z2 = polyfit(S1.CVRods{3}(2).Xs, z2,3);
    
    
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
