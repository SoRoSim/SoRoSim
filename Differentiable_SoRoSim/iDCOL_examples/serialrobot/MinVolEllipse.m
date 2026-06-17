function [A, c] = MinVolEllipse(P, tolerance,sc)

[d, N] = size(P);

% normalize points
mu = mean(P, 2);
Pc = P - mu;

scale = max(vecnorm(Pc, 2, 1));
if scale < eps
    scale = 1;
end
Pn = Pc / scale;

Q = [Pn; ones(1, N)];

u = ones(N,1) / N;
err = inf;

while err > tolerance
    X = Q * diag(u) * Q';
    X = (X + X') / 2;

    reg = 1e-12 * trace(X) / (d+1);
    X = X + reg * eye(d+1);

    Y = X \ Q;
    M = sum(Q .* Y, 1)';

    [maximum, j] = max(M);

    step_size = (maximum - d - 1) / ((d + 1) * (maximum - 1));
    step_size = max(0, min(1, step_size));

    new_u = (1 - step_size) * u;
    new_u(j) = new_u(j) + step_size;

    err = maximum - (d + 1);
    u = new_u;
end

U = diag(u);
cn = Pn * u;

S = Pn * U * Pn' - (Pn * u) * (Pn * u)';
S = (S + S') / 2;

regS = 1e-12 * trace(S) / d;
S = S + regS * eye(d);

An = (1/d) * (S \ eye(d));

% transform back
c = mu + scale * cn;
A = An / (scale^2);

A = (A + A') / 2;
% scale = 1.05;      % 5% larger semi-axis lengths
A = A / sc^2;
end