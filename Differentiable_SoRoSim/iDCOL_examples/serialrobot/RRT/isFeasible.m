function feasible = isFeasible(Linkage,q,env)
    g = Linkage.FwdKinematics(q,Linkage.CP1(1));
    Z = g(7:4:end,4);
    if min(Z) > env.hole1(3) || max(Z)<env.hole1(3)
        feasible = false; % Set feasible to false if the condition is not met
        return
    end
    idx = find(Z>= env.hole1(3),1,'last');
    X1 = Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx)+(env.hole1(3) - Z(idx))*(Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx+1) -Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx))/(Z(idx+1)-Z(idx));
    g = Linkage.FwdKinematics(q,Linkage.CP1(2));
    Z = g(7:4:end,4);
    if min(Z) > env.hole2(3) || max(Z) < env.hole2(3)
        feasible = false;
        return
    end
    idx = find(Z>= env.hole2(3),1,'first');
    idx =idx-1;
    X2 = Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx)+(env.hole1(3) - Z(idx))*(Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx+1) -Linkage.CVRods{Linkage.CP1(1)}(2).Xs(idx))/(Z(idx+1)-Z(idx));
    [p, ~] = position_derivatives(Linkage, [q;zeros(Linkage.nact+Linkage.CLprecompute.nCLp,1); X1;X2], Linkage.CP1);
    
    if norm(env.hole1(1:2)'  - p(1:2))^2 - 1*(env.Radius - 0.0009)^2>0 || norm(env.hole2(1:2)' - p(4:5))^2 - 1*(env.Radius - 0.0009)^2>0
        feasible = false;
        return
    end
    feasible = true;
end
