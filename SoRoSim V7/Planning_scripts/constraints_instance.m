function [c, ceq, Dc, Dceq] = constraints_instance(S1, x,constrain_surface)
    qul = [x(1:S1.ndof); x(S1.ndof+13:S1.ndof+18)];
    uq = x(S1.ndof+1:S1.ndof+12);
    xbar1 = x(S1.ndof+19);
    xbar2 = x(S1.ndof+20);
    c = [];
    Dc = [];
    % lsqoptions = optimoptions('lsqlin','Display','off');
    magnifier = 1;
    Dceq = zeros(80,66);
    [ceq, Dceq(1:66,1:66)] = Equilibrium(S1,qul,uq, magnifier);
    % Xs = S1.CVTwists{1}(2).Xs;
    % V = [ones(length(Xs),1) Xs Xs.^2];
    % g = S1.FwdKinematics(qu_uq_l(1:S1.ndof));
    % 
    % x1 = g(4+1:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    % y1 = g(4+2:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    % z1 = g(4+3:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    % poly_x = (V'*V)\V'*x1;
    % poly_y = (V'*V)\V'*y1;
    % poly_z = (V'*V)\V'*z1;
    % xh1 = [poly_x'*[1; xbar1; xbar1^2] poly_y'*[1; xbar1; xbar1^2] poly_z'*[1; xbar1; xbar1^2]];
    % 
    % Xs = S1.CVTwists{1}(2).Xs;
    % V = [ones(length(Xs),1) Xs Xs.^2];
    % 
    % x2 = g(4*(length(S1.CVTwists{1}(2).Xs) + 4)+1:4:4*(length(S1.CVTwists{1}(2).Xs)+length(S1.CVTwists{3}(2).Xs)+4),4);
    % y2 = g(4*(length(S1.CVTwists{1}(2).Xs) + 4)+2:4:4*(length(S1.CVTwists{1}(2).Xs)+length(S1.CVTwists{3}(2).Xs)+4),4);
    % z2 = g(4*(length(S1.CVTwists{1}(2).Xs) + 4)+3:4:4*(length(S1.CVTwists{1}(2).Xs)+length(S1.CVTwists{3}(2).Xs)+4),4);
    % poly_x = (V'*V)\V'*x2;
    % poly_y = (V'*V)\V'*y2;
    % poly_z = (V'*V)\V'*z2;
    % xh2 = [poly_x'*[1; xbar2; xbar2^2] poly_y'*[1; xbar2; xbar2^2] poly_z'*[1; xbar2; xbar2^2]];
    % eq1 = norm([constrain_surface.hole_1 constrain_surface.height] - xh1);
    % eq2 = norm([constrain_surface.hole_2 constrain_surface.height] - xh2);
    % % ceq = [ceq; eq1; eq2];
    % c = [eq1 - 0.5*(constrain_surface.radius - 0.01); eq2 - 0.5*(constrain_surface.radius - 0.01)];
end