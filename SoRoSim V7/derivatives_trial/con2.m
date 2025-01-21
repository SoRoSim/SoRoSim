function [con2, dcon2] = con2(S1, x_opt, constraint_surface)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    xbar1 = x_opt(79);
    % V = [ones(length(Xs),1) Xs Xs.^2];
    Xs = S1.CVTwists{1}(2).Xs;
    g = S1.FwdKinematics(x_opt(1:S1.ndof));

    % x1 = g(4+1:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    % y1 = g(4+2:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    % z1 = g(4+3:4:4*(length(S1.CVTwists{1}(2).Xs)+1),4);
    % poly_x = (V'*V)\V'*x1;
    % poly_y = (V'*V)\V'*y1;
    % poly_z = (V'*V)\V'*z1;
    % xh1 = [poly_x'*[1; xbar1; xbar1^2] poly_y'*[1; xbar1; xbar1^2] poly_z'*[1; xbar1; xbar1^2]];
    %% Screw interpolation
    index = find(Xs>=xbar1,1);
    alpha = (xbar1 - Xs(index-1))/(Xs(index)- Xs(index - 1));
    xi = piecewise_logmap(ginv(g(4*(index-1) - 3:4*(index-1),:))*g(4*index-3:4*index,:));
    g_xbar = variable_expmap_gTg(xi*alpha);
    xh1 = g_xbar(1:3,4);

    eq1 = norm([constraint_surface.hole_1 constraint_surface.height] - xh1)^2;
    con2 = eq1 - (constraint_surface.radius + 0.01)^2;

    dcon2 = zeros(size(x_opt));
    xi = S1.ScrewStrain(x_opt(1:S1.ndof),1);
    xi_index = xi(6*index + 1: 6*(index+1));
    xi_pre_index = xi(6*(index-1)+1:6*index);
    xi_xbar = (1-alpha)*xi_pre_index + alpha * xi_index;
    dcon2(79) = -2*([constraint_surface.hole_1 constraint_surface.height]' - xh1)'*g_xbar(1:3,1:3)*xi_xbar(4:6);
end