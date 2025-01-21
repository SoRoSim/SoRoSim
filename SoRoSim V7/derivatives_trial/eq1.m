function [eq1, deq1] = eq1(S1, qu_uq_l)
%%This works
    dxdt = diff(qu_uq_l(:,1:S1.ndof));
    eq1 = sum(sum(dxdt.^2, 2));
    deq1 = zeros(10,80);

    deq1(2:end-1, 1:S1.ndof) = 2 * dxdt(1:end-1, :) - 2 * dxdt(2:end, :);
    deq1(1, 1:S1.ndof) = -2 * dxdt(1, :);
    deq1(end, 1:S1.ndof) = 2 * dxdt(end, :);
end
