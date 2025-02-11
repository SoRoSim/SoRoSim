function [n_k,index_q_u,index_q_k,index_u_k,index_u_u] = JointConstraintPrecompute(S)

index_q_k = zeros(1, S.n_jact); % Maximum size is S.n_jact
index_u_u = zeros(1, S.n_jact); % Maximum size is S.n_jact

count = 0; % Single counter
for i = 1:S.n_jact
    if ~S.WrenchControlled(i)
        count = count + 1; % Increment the counter
        index_q_k(count) = S.i_jactq(i);
        index_u_u(count) = i;
    end
end

% Trim unused entries
index_q_k = index_q_k(1:count);
index_u_u = index_u_u(1:count);


index_q_u = setdiff(1:S.ndof,index_q_k);
index_u_k = setdiff(1:S.nact,index_u_u);

n_k = length(index_q_k);

end