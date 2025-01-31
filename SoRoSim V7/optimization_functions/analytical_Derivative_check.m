input = rand(12,1);

func = @(x)Equilibrium(S1, x, input, 1);
[J_num, J_ana] = numericalJacobian(func, x);
%%
x = rand(78,1);

[J_num, J_ana] = numericalJacobian(func, x);
A = J_num;
B = J_ana;
 
% Compute absolute difference matrix
diff_matrix = abs(A - B);
 
% Compute relative difference (ignoring values < 1e-6)
valid_mask = max(abs(A), abs(B)) > 1e-6;  % Ignore small values
relative_diff = zeros(size(A));  % Initialize with zeros
relative_diff(valid_mask) = diff_matrix(valid_mask) ./ max(abs(A(valid_mask)), abs(B(valid_mask)));
 
% Find the maximum relative difference and its index
[max_rel_value, linear_idx] = max(relative_diff(:));
[max_abs_value, abs_linear_idx] = max(diff_matrix(:));

% Convert linear index to row and column indices
[row, col] = ind2sub(size(A), linear_idx);
[row_abs, col_abs] = ind2sub(size(A), abs_linear_idx);


% fid = fopen("optimization_functions\validation.txt", 'a');

% Display the results
fprintf('Max absolute difference: %e at row %d, column %d\n', diff_matrix(row_abs, col_abs), row_abs, col_abs);
fprintf('Max relative difference: %.2f%% at row %d, column %d\n', max_rel_value * 100, row, col);
% fclose(fid);
%%
