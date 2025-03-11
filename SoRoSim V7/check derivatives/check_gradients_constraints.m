function check_gradients_constraints(func_handle, x)
    % Perturbation step for finite difference approximation
    epsilon = 1e-6;  
    n = length(x);  % Number of decision variables

    % Compute analytical gradients using the given function name
     % Convert string to function handle
    [c, ceq, dc_analytical, dceq_analytical] = func_handle(x);
    
    % Initialize finite difference approximations
    dc_finite = zeros(n, length(c));
    dceq_finite = zeros(n, length(ceq));

    % Compute Finite Difference Gradient for Inequality Constraints (c)
    for i = 1:n
        x_perturbed = x;
        x_perturbed(i) = x_perturbed(i) + epsilon;
        [c_perturbed, ~] = func_handle(x_perturbed);
        dc_finite(i, :) = (c_perturbed - c) / epsilon; % Forward difference
    end

    % Compute Finite Difference Gradient for Equality Constraints (ceq)
    for i = 1:n
        x_perturbed = x;
        x_perturbed(i) = x_perturbed(i) + epsilon;
        [~, ceq_perturbed] = func_handle(x_perturbed);
        dceq_finite(i, :) = (ceq_perturbed - ceq) / epsilon; % Forward difference
    end

    % Compare Analytical vs Finite Difference Gradients
    disp(['Checking Gradient Accuracy for Function: ', func2str(func_handle)]);
    
    % Compute Relative Error for Inequality Gradients

    if ~isempty(dc_analytical) && ~isempty(dc_finite) 
        error_dc = abs(dc_analytical - dc_finite) ;
        disp('Maximum relative error in inequality constraint gradients:');
        [max_error, index] = max(error_dc(:));
        disp(max_error(:));
        [row, col] = ind2sub(size(error_dc), index);
        disp(['Largest discrepancy found in **inequality constraint gradients** at x(', num2str(row), ', ', num2str(col), ')']);
        
    else
        error_dc = 1e-6;
    end
    if ~isempty(dceq_analytical) && ~isempty(dceq_finite)
        % Compute Relative Error for Equality Gradients
        error_dceq = abs(dceq_analytical - dceq_finite);
        disp(' Maximum relative error in equality constraint gradients:');
        [max_error, index] = max(error_dceq(:));
        disp(max_error(:));
        [row, col] = ind2sub(size(error_dceq), index);

        disp(['Largest discrepancy found in **equality constraint gradients** at x(', num2str(row), ', ', num2str(col), ')']);
    else
        error_dceq = 1e-6;
    end
    % Display Gradient Comparison Summary
    if max(error_dc(:)) < 1e-4 && max(error_dceq(:)) < 1e-4
        disp('Gradients are accurate (within numerical tolerance).');
    else
        disp('WARNING: Significant discrepancies found in gradients!');
        disp('   -> Check for errors in analytical derivatives.');
    end
end
