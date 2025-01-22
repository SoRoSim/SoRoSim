function [ceq, Dceq] = constraints_instance(S1, x)
    qul = [x(1:S1.ndof); x(S1.ndof+13:S1.ndof+18)];
    uq = x(S1.ndof+1:S1.ndof+12);
    xbar1 = x(S1.ndof+19);
    xbar2 = x(S1.ndof+20);
    c = [];
    Dc = [];
    % lsqoptions = optimoptions('lsqlin','Display','off');
    magnifier = 1;
    Dceq = zeros(80,66);
    [Res, Jac] = Equilibrium(S1,qul,uq, magnifier);
    
    ceq = Res;
    B = S1.ActuationMatrix(qul(1:S1.ndof));
    Dceq = [Jac(:,1:60), [B; zeros(6,12)], Jac(:,61:66)];
end