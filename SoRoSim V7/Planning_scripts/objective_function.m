function E = objective_function(S1, qu_uq_l,g_desired)
    dxdt = diff(qu_uq_l(:,1:S1.ndof));
    qT = qu_uq_l(end,1:S1.ndof);
    gs = S1.FwdKinematics(qT);
    g_platformT = gs(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    E = sum(sum(dxdt.^2, 2)) + 0.5e1* norm(piecewise_logmap(ginv(g_desired)*g_platformT));
end