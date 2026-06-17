function [R, JR] = iEulerResidueJacobian(qd_next, Linkage, q, qd, dt, u)

q_next   = q + dt*qd_next;
qdd_next = (qd_next - qd)/dt;

if nargout == 1
    % Residual only: this should call a cheap RNEA / ID function
    [ID, tau] = RNEA(Linkage, 0, q_next, qd_next, qdd_next, u);

    R = ID - tau;
    return;
end

% Residual + Jacobian
[ID,tau,dID_dq,dID_dqd,dID_dqdd,dtau_dq,dtau_dqd,~,~,~] = ...
    dRNEA(Linkage, 0, q_next, qd_next, qdd_next, u);

R = ID - tau;

JR = dID_dqdd/dt + dID_dqd + dt*dID_dq - (dtau_dqd + dt*dtau_dq);

end