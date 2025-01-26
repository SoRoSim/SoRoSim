function [J, dJ] = Objective3(Linkage, x, g_des_initial) 
    matrix_form = reshape(x, [78, 10])';
    dxdt = diff(matrix_form(:,1:Linkage.ndof));
    eq1 = sum(sum(dxdt.^2, 2));
    
    qT = matrix_form(end, 1:Linkage.ndof);
    gT = Linkage.FwdKinematics(qT);
    g_T = gT(4*(length(Linkage.CVRods{1}(2).Xs)+2)+1:4*(length(Linkage.CVRods{1}(2).Xs)+3),:);
    e2 = piecewise_logmap(ginv(g_des_initial)*g_T);
    eq2 = norm(e2)^2;
    deq1 = zeros(10,78);

    deq1(2:end-1, 1:Linkage.ndof) = 2 * dxdt(1:end-1, :) - 2 * dxdt(2:end, :);
    deq1(1, 1:Linkage.ndof) = -2 * dxdt(1, :);
    deq1(end, 1:Linkage.ndof) = 2 * dxdt(end, :);
    deq2 = zeros(10,78);
    [~,Teq2] = variable_expmap_gTg(e2);
    Ad_exp_eq2 = dinamico_Adjoint(ginv(g_des_initial)*g_T);
    J_platform = Linkage.Jacobian(x(1:Linkage.ndof), 2);
    J_platform = J_platform(7:12,:);
    deq2(1,1:60) = 2*piecewise_logmap(ginv(g_des_initial)*g_T)' * (Teq2\(Ad_exp_eq2*J_platform));

    J = eq1 + eq2;
    deq1 = deq1';
    deq1 = deq1(:);
    deq2 = deq2';
    deq2 = deq2(:);
    dJ = deq1 + deq2;
end