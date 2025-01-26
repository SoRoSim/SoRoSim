function [J_c, J_eq, Jc_ana, Jeq_ana] = numericalJacobianConstraints(func, x, epsilon)
    if nargin < 3
        epsilon = 1e-6; % Default step size
    end
    n = length(x);
    [~ ,~, Jc_ana, Jeq_ana] = func(x);
    J_eq = zeros(size(Jeq_ana));
    J_c = zeros(size(Jc_ana));
    for i = 1:n
        x_forward = x;
        x_backward = x;
        x_forward(i) = x_forward(i) + epsilon;
        x_backward(i) = x_backward(i) - epsilon;
        [c_forward, ceq_forward, ~, ~] = func(x_forward);
        [c_backward, ceq_backward, ~, ~] = func(x_backward);
        if ~isempty(J_c)
            J_c(i,:) = (c_forward - c_backward) / (2 * epsilon);
        end
        if ~isempty(J_eq)
            J_eq(i,:) = (ceq_forward - ceq_backward)/ (2 * epsilon);
        end
    end
end