function [con2, dcon2] = con2(S1, x_opt, constraint_surface)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    xbar1 = x_opt(79);
    Xs = S1.CVTwists{1}(2).Xs;
    g = S1.FwdKinematics(x_opt(1:S1.ndof));
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