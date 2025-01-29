function [g_interpolated, xi_interpolated, J_interpolated] = Screw_interpolate(Linkage,xbar, q,Linkindex)
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
    g = Linkage.FwdKinematics(q,Linkindex);
    xi = Linkage.ScrewStrain(q,Linkindex);
    J = Linkage.Jacobian(q,Linkindex);
    if xbar == T(idx)
        g_interpolated = g(4*idx-3:4*idx,:);
        xi_interpolated = xi(6*idx-5:6*idx);
        J_interpolated = J(6*idx-5:6*idx);
        return;
    end
    if idx ==7
    disp(xbar)
    end

    t_i = T(idx);
    t_next = T(idx + 1);
    g_i = g(4*idx-3:4*idx,:);
    g_next = g(4*(idx+1)-3:4*(idx+1),:);

    % Compute interpolation parameter alpha
    alpha = (xbar - t_i) / (t_next - t_i);

    % Relative transformation
    g_rel = g_i \ g_next; % Equivalent to inv(g_i) * g_next

    % Logarithmic map
    xi_rel = piecewise_logmap(g_rel);

    % Scale the twist
    xi_alpha = alpha * xi_rel;

    % Exponential map
    g_alpha = variable_expmap_g(xi_alpha);

    % Interpolated transformation
    g_interpolated = g_i * g_alpha;

    xi_index = xi(6*idx + 1: 6*(idx+1));
    xi_pre_index = xi(6*(idx-1)+1:6*idx);
    xi_interpolated = (1-alpha)*xi_pre_index + alpha * xi_index;
    
    J_index = J(6*idx + 1: 6*(idx+1),:);
    J_pre_index = J(6*(idx-1)+1:6*idx,:);
    J_interpolated = (1-alpha)*J_pre_index + alpha * J_index;
end
