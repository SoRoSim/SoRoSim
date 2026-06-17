function Q = fit_quadric(Vworld,gamma)
    mu = mean(Vworld,1);
    X = Vworld - mu;
    s = max(vecnorm(X,2,2));
    X = X / s;
    
    x = X(:,1); y = X(:,2); z = X(:,3);
    
    M = [x.^2, y.^2, z.^2, ...
         x.*y, x.*z, y.*z, ...
         x, y, z, ...
         ones(size(x))];
    
    [~,~,V] = svd(M,0);
    p = V(:,end); % this produces the optimal coefficients for the quadric, but we transformed the points
    
    Q_hat = [ p(1)   p(4)/2 p(5)/2 p(7)/2;
  p(4)/2 p(2)   p(6)/2 p(8)/2;
  p(5)/2 p(6)/2 p(3)   p(9)/2;
  p(7)/2 p(8)/2 p(9)/2 p(10) ];

    T = [1/s  0    0   -mu(1)/s;
         0    1/s  0   -mu(2)/s;
         0    0    1/s -mu(3)/s;
         0    0    0    1];
    
    Q = T' * Q_hat * T;
    
    % enlarge slightly
    
    A = Q(1:3,1:3);
    b = Q(1:3,4);
    c = -A\b;
    
    H = [gamma*eye(3), (1-gamma)*c;
         0 0 0 1];
    
    Q_big = inv(H)' * Q * inv(H);
    Q_big = Q_big / norm(Q_big);