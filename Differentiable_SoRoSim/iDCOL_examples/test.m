%% Half-space covering of random 3D points (fixed: bounded hull)
clear; clc; close all;
rng(0);

%% Parameters
n = 200;     % number of points
m = 8;      % number of half-spaces (>=6 recommended)

%% Random points
A = randn(3,3);
P = randn(n,3) * A;

%% Uniform normals on sphere via Fibonacci lattice
N = fibonacci_sphere(m);   % m x 3 unit vectors

%% Half-spaces a_i^T x <= b_i with b_i = support function of P
b = max(P * N', [], 1)';

%% Vertex enumeration of {x : N x <= b}
V = halfspace_vertices(N, b);
K = convhull(V(:,1), V(:,2), V(:,3));

%% Plot
figure('Color','w','Position',[100 100 900 700]); hold on; grid on; axis equal;
view(135,25);
xlabel('x'); ylabel('y'); zlabel('z');
title(sprintf('%d points covered by %d half-spaces', n, m));

scatter3(P(:,1), P(:,2), P(:,3), 18, 'k', 'filled', 'MarkerFaceAlpha',0.6);

trisurf(K, V(:,1), V(:,2), V(:,3), ...
        'FaceColor',[0.2 0.5 0.9], 'FaceAlpha',0.20, ...
        'EdgeColor',[0.1 0.3 0.7], 'LineWidth',1.0);

c = mean(P,1);
quiver3(repmat(c(1),m,1), repmat(c(2),m,1), repmat(c(3),m,1), ...
        N(:,1), N(:,2), N(:,3), 1.0, 'r', 'LineWidth',1);

legend({'points','bounding polyhedron','normals'},'Location','best');

%% ---------- helpers ----------
function N = fibonacci_sphere(m)
    % Quasi-uniform points on unit sphere
    k  = (0:m-1)';
    phi   = acos(1 - 2*(k+0.5)/m);          % polar
    theta = pi*(1+sqrt(5))*k;               % azimuth (golden angle)
    N = [sin(phi).*cos(theta), sin(phi).*sin(theta), cos(phi)];
end

function V = halfspace_vertices(N, b)
    m  = size(N,1);
    C  = nchoosek(1:m,3);
    tol = 1e-7;
    pts = zeros(0,3);
    for k = 1:size(C,1)
        idx = C(k,:);
        A3  = N(idx,:);
        if abs(det(A3)) < tol, continue; end
        x = A3 \ b(idx);
        if all(N*x <= b + 1e-6)
            pts(end+1,:) = x'; %#ok<AGROW>
        end
    end
    V = uniquetol(pts, 1e-6, 'ByRows', true);
    if size(V,1) < 4
        error('Polyhedron unbounded or degenerate. Increase m.');
    end
end