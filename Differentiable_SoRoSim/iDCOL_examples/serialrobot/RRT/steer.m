function [state_new, q_a, x] = steer(Linkage,T_near, state_rand, stepSize)
    Omega = zeros(12,1);
    state_near = T_near.state;
    Omega(1:6) = piecewise_logmap(ginv(state_near(1:4,:))*state_rand(1:4,:));
    Omega(7:12) = piecewise_logmap(ginv(state_near(5:8,:))*state_rand(5:8,:));

    nrm = norm(Omega);
    if nrm < 1e-3
        state_new = state_near;
    else
        state_new(1:4,:) = state_near(1:4,:)*variable_expmap_g(stepSize*Omega(1:6));
        state_new(5:8,:) = state_near(5:8,:)*variable_expmap_g(stepSize*Omega(7:12));
    end
    
    q_a = Inverse_kinematics(Linkage,state_new,T_near.q_a);
    [q, u,lambda]  = Linkage.statics(T_near.x,q_a,"plot",false);
    x = [q(8:end-7);u;lambda];
end