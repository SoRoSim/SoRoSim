% Plotting the rope at q
function [constraint_surface, root1, root2] = find_constraint(S1,q, constraint_height)
    g = S1.FwdKinematics(q);
    Xs1 = S1.CVRods{1}(2).Xs;
    z1 = g(3:4:4*S1.CVRods{1}(2).nip,4);
    x1 = g(1:4:4*S1.CVRods{1}(2).nip,4);
    y1 = g(2:4:4*S1.CVRods{1}(2).nip,4);
    constraint_surface.radius = 0.05;
    constraint_surface.height = constraint_height;

    index = find(z1<=constraint_height,1);
    alpha = (constraint_height - z1(index-1))/(z1(index)- z1(index - 1));
    root1 = Xs1(index-1)*(1-alpha) + Xs1(index)*alpha;
    xh = x1(index-1)*(1-alpha) + x1(index)*alpha;
    yh = y1(index-1)*(1-alpha) + y1(index)*alpha;
    constraint_surface.hole_1 = [xh; yh; constraint_surface.height];

    Xs2 = S1.CVRods{3}(2).Xs;
    z2 = g(4*(S1.CVRods{1}(2).nip+5)-1:4:end-8,4);
    x2 = g(4*(S1.CVRods{1}(2).nip+5)-3:4:end-8,4);
    y2 = g(4*(S1.CVRods{1}(2).nip+5)-2:4:end-8,4);

    index = find(z2>=constraint_height,1);
    alpha = (constraint_height - z2(index-1))/(z2(index)- z2(index - 1));
    root2 = Xs2(index-1)*(1-alpha) + x2(index)*alpha;
    xh = x2(index-1)*(1-alpha) + x2(index)*alpha;
    yh = y2(index-1)*(1-alpha) + y2(index)*alpha;
    

    constraint_surface.hole_2 = [xh; yh; constraint_height];
end