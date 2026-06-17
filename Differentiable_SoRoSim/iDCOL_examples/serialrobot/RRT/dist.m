function d = dist(q1, q2)
% We cannot  compare the distance between two nodes based on SE(3) distance
% of the tooltip. the shape of the deformable object or the manipulator arm
% might be different. So comparison is made based on the strain parameters
    W = diag(ones(length(q1),1));   % you can weight some strains more
    d = sqrt((q1 - q2)' * W * (q1 - q2));
end