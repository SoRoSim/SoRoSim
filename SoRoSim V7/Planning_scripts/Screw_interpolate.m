% function [g_xbar, xi_xbar] = Screw_interpolate(Linkage, xbar, q, LinkIndex)
%     Xs = Linkage.CVRods{LinkIndex}(2).Xs;
%     g = Linkage.FwdKinematics(q,LinkIndex);
%     xi = Linkage.ScrewStrain(q,LinkIndex);
%     if xbar ==0
%         g_xbar = g(1:4,1:4);
%         xi_xbar = Linkage.ScrewStrain(q,LinkIndex);
%     else
%         index = find(Xs>=xbar,1);
%         alpha = (xbar - Xs(index-1))/(Xs(index)- Xs(index - 1));
%         xi_interp = piecewise_logmap(ginv(g(4*(index-1) - 3:4*(index-1),:))*g(4*index-3:4*index,:));
%         g_xbar = g(4*index-3:4*index,:)*variable_expmap_g(xi_interp*alpha);
%         xi_index = xi(6*index + 1: 6*(index+1));
%         xi_pre_index = xi(6*(index-1)+1:6*index);
%         xi_xbar = (1-alpha)*xi_pre_index + alpha * xi_index;
%     end
% end
function g_interpolated = Screw_interpolate(Linkage,xbar, q,Linkindex)
    % Geodesic interpolation in SE(3).
    % Inputs:
    % t  - Query point (0 <= t <= 1)
    % T  - Array of discrete points [t1, t2, ..., tN]
    % G  - Cell array of SE(3) matrices {g1, g2, ..., gN}
    % Output:
    % g_interpolated - Interpolated transformation matrix in SE(3)
    T = Linkage.CVRods{Linkindex}(2).Xs;
    % Find the two nearest points t_i and t_{i+1}
    idx = find(T <= xbar, 1, 'last');
    g = Linkage.Fwdkinematics(q);
    
    t_i = T(idx);
    t_next = T(idx + 1);
    g_i = g(4*idx-3:4*idx,:);
    g_next = g(4*(idx+1)-3:4*(idx+1),:);

    % Compute interpolation parameter alpha
    alpha = (t - t_i) / (t_next - t_i);

    % Relative transformation
    g_rel = g_i \ g_next; % Equivalent to inv(g_i) * g_next

    % Logarithmic map
    xi = piecewise_logmap(g_rel);

    % Scale the twist
    xi_alpha = alpha * xi;

    % Exponential map
    g_alpha = variable_expmap_g(xi_alpha);

    % Interpolated transformation
    g_interpolated = g_i * g_alpha;
end
