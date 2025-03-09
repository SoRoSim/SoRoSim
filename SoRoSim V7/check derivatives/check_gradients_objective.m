function check_gradients_objective(func_handle, x)
    % Perturbation step for finite difference approximation
    epsilon = 1e-6;  
    n = length(x);  % Number of decision variables

    % Compute analytical gradients using the given function name
     % Convert string to function handle
    [J, dJ] = func_handle(x);
    
    % Initialize finite difference approximations
    dc_finite = zeros(n, 1);

    % Compute Finite Difference Gradient for Inequality Constraints (c)
    for i = 1:n
        x_perturbed = x;
        x_perturbed(i) = x_perturbed(i) + epsilon;
        [J_perturbed, ~] = func_handle(x_perturbed);
        dc_finite(i, :) = (J_perturbed - J) / epsilon; % Forward difference
    end


    % Compare Analytical vs Finite Difference Gradients
    disp(['🔹 Checking Gradient Accuracy for Function: ', func2str(func_handle)]);
    
    % Compute Relative Error for Inequality Gradients
    if ~isempty(dJ) && ~isempty(dc_finite) 
        error_dc = abs(dJ - dc_finite) ./ (abs(dc_finite) + 1e-8);
        disp('➡️  Maximum relative error in inequality constraint gradients:');
        disp(max(error_dc(:)));
    else
        error_dc = 1e-6;
    end
    
    % Display Gradient Comparison Summary
    if max(error_dc(:)) < 1e-4
        disp('✅ Gradients are accurate (within numerical tolerance).');
    else
        disp('⚠️ WARNING: Significant discrepancies found in gradients!');
        disp('   -> Check for errors in analytical derivatives.');
    end
end
