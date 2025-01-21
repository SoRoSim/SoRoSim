function [c, ceq] = constraints(S1, qu_uq_l,n_points,constrain_surface, g_0)
    c = [];
    qul = [qu_uq_l(:,1:S1.ndof)'; qu_uq_l(:,S1.ndof+13:S1.ndof+18)']';
    uq = qu_uq_l(:,S1.ndof+1:S1.ndof+12);
    x1 = qu_uq_l(:,S1.ndof+19);
    x2 = qu_uq_l(:,S1.ndof+20);
    ceq = [];
    for i =1:n_points
        g = S1.FwdKinematics(qu_uq_l(i,1:S1.ndof));
        X1s = S1.CVTwists{1}(2).Xs;
        X2s = S1.CVTwists{3}(2).Xs;
        X1 = polyfit(X1s, g(5:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
        Y1 = polyfit(X1s, g(6:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
        Z1 = polyfit(X1s, g(7:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4),3);
        xh1 = [polyval(X1,x1(i)) polyval(Y1,x1(i)) polyval(Z1,x1(i))];
        eq1 = norm([constrain_surface.hole_1 constrain_surface.height] - xh1);
        X2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+1:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
        Y2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+2:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
        Z2 = polyfit(X2s, g(4*(length(S1.CVTwists{1}(2).Xs)+4)+3:4:4*(length(S1.CVTwists{1}(2).Xs)+4+length(S1.CVTwists{3}(2).Xs)),4),3);
        xh2 = [polyval(X2,x2(i)) polyval(Y2,x2(i)) polyval(Z2,x2(i))];
        eq2 = norm([constrain_surface.hole_2 constrain_surface.height] - xh2);
        lsqoptions = optimoptions('lsqlin','Display','off');
        magnifier = 1;
        eq3 = Equilibrium_optim(S1,qul(i,:)',uq(i,:)', magnifier, lsqoptions);
        ceq = [ceq; eq3];
        c = [c;eq1 - 0.5*(constrain_surface.radius - 0.01); eq2 - 0.5*(constrain_surface.radius - 0.01)];
    end
    
    q0 = qu_uq_l(1,1:S1.ndof);
    gs = S1.FwdKinematics(q0);
    g_platform0 = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    eq_5 = piecewise_logmap(ginv(g_platform0)*g_0);
    ceq = [ceq; eq_5];
end