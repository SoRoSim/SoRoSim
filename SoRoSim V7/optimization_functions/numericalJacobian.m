function [J, J_ana] = numericalJacobian(func, x, epsilon)
    if nargin < 3
        epsilon = 1e-6; % Default step size
    end
    n = length(x);
    J = zeros(length(func(x)), n);
    for i = 1:n
        x_forward = x;
        x_backward = x;
        x_forward(i) = x_forward(i) + epsilon;
        x_backward(i) = x_backward(i) - epsilon;
        J(:, i) = (func(x_forward) - func(x_backward)) / (2 * epsilon);
    end
    [~,J_ana] =  func(x);
end