function path = backtrackPath(T, idx)
    q_dim = numel(T(1).q);
    path  = T(idx).q;   % m x 1
    parent = T(idx).parent;

    while parent ~= 0
        path = [T(parent).q, path]; 
        parent = T(parent).parent;
    end
end