function [g_xbar, xi_xbar] = Screw_interpolate(Linkage, xbar, q, LinkIndex)
    Xs = Linkage.CVRods{LinkIndex}(2).Xs;
    g = Linkage.FwdKinematics(q,LinkIndex);
    xi = Linkage.ScrewStrain(q,LinkIndex);
    if xbar ==0
        g_xbar = g(1:4,1:4);
        xi_xbar = Linkage.ScrewStrain(q,LinkIndex);
    else
        index = find(Xs>=xbar,1);
        alpha = (xbar - Xs(index-1))/(Xs(index)- Xs(index - 1));
        xi_interp = piecewise_logmap(ginv(g(4*(index-1) - 3:4*(index-1),:))*g(4*index-3:4*index,:));
        g_xbar = g(4*index-3:4*index,:)*variable_expmap_g(xi_interp*alpha);
        xi_index = xi(6*index + 1: 6*(index+1));
        xi_pre_index = xi(6*(index-1)+1:6*index);
        xi_xbar = (1-alpha)*xi_pre_index + alpha * xi_index;
    end
end