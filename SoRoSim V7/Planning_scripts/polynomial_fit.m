function poly = polynomial_fit(S1,x1)
% Input: S1 (Sorosimlinkage), x1(position at each computational point)
% Finds a third order polynomial that best fits the centerline of a soft
% material
    %% Using least squares closed form
    Xs = S1.CVTwists{1}(2).Xs;
    V = [ones(length(Xs),1) Xs Xs.^2];
    poly = (V'*V)\V'*x1;
end