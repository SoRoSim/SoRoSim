function model = fitConvexSuperquadricBound(X, p, makePlot)
%FITCONVEXSUPERQUADRICBOUND Fit a smooth convex implicit bound to 3D points.
%
%   model = fitConvexSuperquadricBound(X)
%   model = fitConvexSuperquadricBound(X, p)
%   model = fitConvexSuperquadricBound(X, p, makePlot)
%
% Inputs
%   X        : N-by-3 array of 3D points
%   p        : positive integer, default = 2
%              Implicit surface is
%                 f(x) = sum_i (y_i / r_i)^(2p) - 1
%              where y = R' * (x - c)
%              p = 1 gives an ellipsoid
%              p > 1 gives a boxier but still smooth convex shape
%   makePlot : logical, default = false
%
% Output
%   model is a struct with fields:
%     .center          3-by-1 center
%     .R               3-by-3 rotation matrix
%     .radii           3-by-1 axis scales
%     .p               exponent parameter
%     .implicit        function handle, accepts N-by-3 points
%     .allInside       true if all input points satisfy f(x) <= 0
%     .values          f(X) for input data
%     .surfacePoints   sampled surface points for plotting
%     .surfaceFaces    triangulation faces for plotting
%
% Example
%   X = randn(500,3) * diag([2 0.7 0.4]);
%   model = fitConvexSuperquadricBound(X, 3, true);
%   vals = model.implicit(X);
%   all(vals <= 1e-10)
%
% Notes
%   1) This returns a smooth convex bound, not necessarily the tightest one.
%   2) The function is inexpensive to evaluate compared with log-sum-exp.
%   3) The bound is aligned to PCA axes of the point cloud.

    if nargin < 2 || isempty(p)
        p = 2;
    end
    if nargin < 3 || isempty(makePlot)
        makePlot = false;
    end

    validateattributes(X, {'numeric'}, {'2d','ncols',3,'real','finite','nonempty'});
    validateattributes(p, {'numeric'}, {'scalar','integer','positive'});
    validateattributes(makePlot, {'logical','numeric'}, {'scalar'});

    n = size(X,1);
    if n < 4
        error('X must contain at least 4 points.');
    end

   
end

function vals = superquadricImplicit(Q, c, R, r, p)
%SUPERQUADRICIMPLICIT Evaluate implicit function on N-by-3 points.
    validateattributes(Q, {'numeric'}, {'2d','ncols',3,'real','finite'});
    Y = (Q - c') * R;
    vals = sum((Y ./ r').^(2*p), 2) - 1;
end

function [R, score, latent] = pcaAxes(Xc)
%PCAAXES PCA without Statistics Toolbox dependency.
% Returns principal axes as columns of R.
    C = (Xc' * Xc) / max(size(Xc,1)-1, 1);
    [V, D] = eig((C + C')/2);
    [latent, idx] = sort(diag(D), 'descend');
    R = V(:, idx);

    % Ensure right-handed frame
    if det(R) < 0
        R(:,3) = -R(:,3);
    end
    score = Xc * R;
end

function r = tightenRadii(Y, rInit, p)
%TIGHTENRADII Light coordinate-descent tightening under containment.
% Keeps all points inside while slightly shrinking the bound.
    r = rInit;
    maxIter = 3;
    for it = 1:maxIter
        for j = 1:3
            lo = 1e-12;
            hi = r(j);

            % Feasible lower bound search by bisection
            for k = 1:40
                mid = 0.5 * (lo + hi);
                rTest = r;
                rTest(j) = mid;
                vals = sum((Y ./ rTest').^(2*p), 2);
                if all(vals <= 1 + 1e-12)
                    hi = mid;
                else
                    lo = mid;
                end
            end
            r(j) = hi;
        end
    end
end

function [V, F] = sampleSuperquadricSurface(c, R, r, p, nu, nv)
%SAMPLESUPERQUADRICSURFACE Sample the implicit surface for plotting.
%
% Surface in local frame is parameterized by:
%   x = r1 * sgn(cos u) * |cos u|^(1/p) * sgn(cos v) * |cos v|^(1/p)
%   y = r2 * sgn(cos u) * |cos u|^(1/p) * sgn(sin v) * |sin v|^(1/p)
%   z = r3 * sgn(sin u) * |sin u|^(1/p)
%
% since the surface satisfies:
%   (x/r1)^(2p) + (y/r2)^(2p) + (z/r3)^(2p) = 1

    u = linspace(-pi/2, pi/2, nu);
    v = linspace(-pi, pi, nv);
    [U, Vv] = meshgrid(u, v);

    cu = signedPow(cos(U), 1/p);
    su = signedPow(sin(U), 1/p);
    cv = signedPow(cos(Vv), 1/p);
    sv = signedPow(sin(Vv), 1/p);

    Xl = r(1) * cu .* cv;
    Yl = r(2) * cu .* sv;
    Zl = r(3) * su;

    P = [Xl(:), Yl(:), Zl(:)] * R' + c';
    V = P;

    % Build quad mesh faces
    nRows = size(Xl,1);
    nCols = size(Xl,2);
    F = zeros((nRows-1)*(nCols-1), 4);
    idx = 1;
    for i = 1:nRows-1
        for j = 1:nCols-1
            v1 = sub2ind([nRows, nCols], i,   j);
            v2 = sub2ind([nRows, nCols], i+1, j);
            v3 = sub2ind([nRows, nCols], i+1, j+1);
            v4 = sub2ind([nRows, nCols], i,   j+1);
            F(idx,:) = [v1 v2 v3 v4];
            idx = idx + 1;
        end
    end
end

function y = signedPow(x, a)
%SIGNEDPOW sign(x) * |x|^a
    y = sign(x) .* abs(x).^a;
end