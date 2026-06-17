function [A,b] = BoundingCuboid(V)
c = mean(V,1);           % centroid
Vc = V - c;

[coeff, ~, ~] = pca(Vc); % 3x3, columns are principal axes
R = coeff;               % cuboid orientation

% Coordinates in local cuboid frame
Q = Vc * R;

%% 2) Bounds in local frame
qmin = min(Q, [], 1);
qmax = max(Q, [], 1);

h = (qmax - qmin)/2;          % half-lengths
clocal = (qmax + qmin)/2;     % local center offset

% true cuboid center in world frame
cbox = c + clocal * R';

%% 3) 8 cuboid corners in local frame
corners_local = [
    -h(1) -h(2) -h(3)
     h(1) -h(2) -h(3)
     h(1)  h(2) -h(3)
    -h(1)  h(2) -h(3)
    -h(1) -h(2)  h(3)
     h(1) -h(2)  h(3)
     h(1)  h(2)  h(3)
    -h(1)  h(2)  h(3)
];

%% 4) Transform corners to world frame
corners_world = corners_local * R' + cbox;

%% 5) Faces of the cuboid
faces = [
    1 2 3 4   % bottom
    5 6 7 8   % top
    1 2 6 5   % side
    2 3 7 6   % side
    3 4 8 7   % side
    4 1 5 8   % side
];

% %% 6) Plot points + cuboid surface using patch
% figure;
% plot3(V(:,1), V(:,2), V(:,3), '.', 'MarkerSize', 8); hold on;
% 
% p = patch('Vertices', corners_world, ...
%           'Faces', faces, ...
%           'FaceColor', [0.2 0.6 0.8], ...
%           'FaceAlpha', 0.20, ...
%           'EdgeColor', 'k', ...
%           'LineWidth', 1.2);
% 
% axis equal;
% grid on;
% view(3);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('PCA-oriented bounding cuboid');
% 
% hold on

%%
A = [ R'
     -R' ];

b = [ h(:) + R' * cbox(:)
      h(:) - R' * cbox(:) ];
end