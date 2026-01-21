%% iDCOL in SoRoSim: contact demo (two polytopes)
% Goal: move Body 2 along x, run narrowphase (broadphase done within), visualize x* and alpha*.
% Things you will likely tweak:
%   - beta (smoothing), A1/b1, A2/b2 (polytopes), motion profile, video options.

clear; close all; clc;

%% ----------------------- User knobs -----------------------
shape_id   = 2;
beta       = 30;          % high beta can make solve harder / less robust
N          = 120;         % animation steps
x_start    = 5;           % initial x position of body 2
x_end      = -5;          % final x position of body 2 (sweep direction)
pause_dt   = 0.01;        % visualization pause

make_video = true;
vidfile    = 'idcol_contact_demo.mp4';  % '.avi' also ok
fps        = 30;

fig_pos_px = [100 100 1280 720];        % even width/height for H.264
view_dir   = [0 0];                     % view(az,el)

%% ----------------------- Define polytopes -----------------------
[A1, b1] = demo_polytope_1();
[A2, b2] = demo_polytope_2();

%% ----------------------- Create bodies + pair -----------------------
params1 = idcol_make_params(shape_id, beta, A1, b1);
params2 = idcol_make_params(shape_id, beta, A2, b2);

B1 = SorosimContactBody(1, shape_id, params1);
B2 = SorosimContactBody(2, shape_id, params2);

P12 = SorosimContactPair(B1, B2, 1);

%% ----------------------- Visualization setup -----------------------
[fig, ax] = setup_scene(fig_pos_px, view_dir);

B1.attachHandles(ax);
B2.attachHandles(ax);

style_body(B1, [1.0 0.6 0.2]);
style_body(B2, [0.2 0.6 1.0]);

% marker for x*
hXstar = plot3(ax, NaN, NaN, NaN, 'o', 'MarkerSize', 8, 'LineWidth', 2);

%% ----------------------- Initial poses -----------------------
g1 = eye(4);
g2 = eye(4);  g2(1:3,4) = [x_start; 0; 0];

set_body_pose(B1, g1);
set_body_pose(B2, g2);

%% ----------------------- Video writer -----------------------
if make_video
    v = VideoWriter(vidfile, 'MPEG-4');          % use 'Motion JPEG AVI' if needed
    v.FrameRate = fps;
    v.Quality   = 95;
    open(v);
end

%% ----------------------- Animate + solve -----------------------
x_traj = linspace(x_start, x_end, N);

for k = 1:N
    g2(1:3,4) = [x_traj(k); 0; 0];
    set_body_pose(B2, g2);

    % solving contact
    [out, success] = P12.solveNarrowPhase(g1, g2);

    % Update UI
    update_title_and_marker(ax, hXstar, P12, success, out, g1, B1);

    drawnow;

    if make_video
        writeVideo(v, getframe(fig));
    end

    pause(pause_dt);
end

if make_video
    close(v);
    fprintf('Saved video: %s\n', fullfile(pwd, vidfile));
end

%% ======================= Helper functions =======================

function [A, b] = demo_polytope_1()
A = 1/sqrt(3)*[ 1   2    1;
                1  -1   -1;
               -1   1   -1;
               -1  -1    1;
               -1  -4   -1;
               -2   1    1;
                1  -3    1;
                1   1    4];
b = [1; 1; 1; 1; 5/3; 5/3; 5/3; 5/3];
end

function [A, b] = demo_polytope_2()
A = 1/sqrt(3)*[ 1   1    1;
                1  -1   -1;
               -1   1   -1;
               -1  -1    1;
               -1  -1   -1;
               -1   1    1;
                1  -1    1;
                1   1   -1];
b = [1; 1; 1; 1; 5/3; 5/3; 5/3; 5/3];
end

function [fig, ax] = setup_scene(fig_pos_px, view_dir)
fig = figure('Color','w','Units','pixels','Position',fig_pos_px);
ax  = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); view(ax, 3);
xlabel(ax,'x'); ylabel(ax,'y'); zlabel(ax,'z');
view(ax, view_dir(1), view_dir(2));
camlight(ax,'headlight'); lighting(ax,'gouraud');
end

function style_body(B, faceColor)
set(B.hGeom, 'EdgeColor','none', 'FaceAlpha',0.35, 'FaceColor', faceColor);
end

function set_body_pose(B, g)
if ismethod(B,'setPose')
    B.setPose(g);
else
    set(B.hT, 'Matrix', g);
end
end

function update_title_and_marker(ax, hXstar, P, success, out, g1, B1)
bp = P.broadphase_active;
ct = P.contact_active;

if bp && success
    title(ax, { ...
        sprintf('$Broadphase = %d \\quad Contact = %d$', bp, ct), ...
        sprintf('$\\alpha^* = %.6g$', out.alpha) ...
        }, 'Interpreter','latex');

    g1C = g1 * B1.g_JC;
    xw  = g1C(1:3,1:3) * out.x(:) + g1C(1:3,4);
    set(hXstar,'XData',xw(1),'YData',xw(2),'ZData',xw(3));
else
    title(ax, sprintf('$Broadphase = %d \\quad Contact = %d$', bp, ct), ...
        'Interpreter','latex');
    set(hXstar, 'XData',NaN, 'YData',NaN, 'ZData',NaN);
end
end
