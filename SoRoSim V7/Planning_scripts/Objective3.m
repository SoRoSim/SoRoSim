function [J, dJ] = Objective3(Linkage, x, g_des_final, n_timesteps)
    %% This is correct
    vars_step = length(x)/n_timesteps;
    matrix_form = reshape(x, [vars_step, n_timesteps])';
    dxdt = diff(matrix_form(:,1:Linkage.ndof));
    eq1 = sum(sum(dxdt.^2, 2));
    deq1 = zeros(length(x),1);
    for i = 1:n_timesteps
        if i == 1
            deq1(1:Linkage.ndof) = -2*dxdt(1,:);
        elseif i == n_timesteps
            deq1(end - vars_step+1:end - vars_step+Linkage.ndof) = 2 * dxdt(end, :);
        else
            deq1((i-1)*vars_step +1: (i-1)*vars_step +Linkage.ndof) = 2 * dxdt(i-1, :) - 2 * dxdt(i, :);
        end
    end


    qT = matrix_form(end, 1:Linkage.ndof);
    gT = Linkage.FwdKinematics(qT);
    g_T = gT(4*(length(Linkage.CVRods{1}(2).Xs)+2)+1:4*(length(Linkage.CVRods{1}(2).Xs)+3),:);
    e2 = piecewise_logmap(ginv(g_des_final)*g_T);
    eq2 = norm(e2)^2;
    deq2 = zeros(10,78);
    [~,Teq2] = variable_expmap_gTg(e2);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des_final)*g_T);
    J_platform = Linkage.Jacobian(x(1:Linkage.ndof), 2);
    J_platform = J_platform(7:12,:);
    deq2(n_timesteps,1:60) = 2*piecewise_logmap(ginv(g_des_final)*g_T)' * (Teq2\(Ad_exp_eq2*J_platform));
    
    deq2 = deq2';
    deq2 = deq2(:);

    % % 
    J = eq1 + eq2;

    dJ = deq1 + deq2;
end