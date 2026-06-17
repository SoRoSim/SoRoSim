function plotq_drone(Linkage, g)
% plotq_temp  Plot the linkage at pose g with environment and checker planes.
%
% Usage:
%   plotq_temp(Linkage)
%   plotq_temp(Linkage, g)

    if nargin < 2
        g = eye(4);
    end

    PlotParameters = Linkage.PlotParameters;

    %% ---------- Figure / axes ----------
    fh = figure;
    ax = axes('Parent', fh); %#ok<NASGU>
    fh.Units = 'normalized';
    fh.OuterPosition = [0 0 1 1];

    set(gca, ...
        'CameraPosition', PlotParameters.CameraPosition, ...
        'CameraTarget',   PlotParameters.CameraTarget, ...
        'CameraUpVector', PlotParameters.CameraUpVector, ...
        'FontSize', 18);

    if PlotParameters.Light
        camlight(PlotParameters.Az_light, PlotParameters.El_light);
    end

    axis equal;
    grid on;
    hold on;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');

    % LaTeX for all text elements
    set(get(gca,'Title'),  'Interpreter','latex');
    set(get(gca,'XLabel'), 'Interpreter','latex');
    set(get(gca,'YLabel'), 'Interpreter','latex');
    set(get(gca,'ZLabel'), 'Interpreter','latex');
    set(gca,'TickLabelInterpreter','latex');
    set(gca,'FontSize',18);

    %% ---------- Draw robot bodies ----------
    for ib = 1:Linkage.nb
        Linkage.RobotBodies(ib).attachHandles(gca);
        Linkage.RobotBodies(ib).setPose(g);
        Linkage.RobotBodies(ib).hGeom.FaceColor = [0 1 0];
        Linkage.RobotBodies(ib).hGeom.FaceAlpha = 1;
    end

    %% ---------- Draw environment bodies ----------
    for ie = 1:Linkage.ne
        Linkage.EnvironmentBodies(ie).attachHandles(gca);
        Linkage.EnvironmentBodies(ie).setPose(eye(4));
        Linkage.EnvironmentBodies(ie).hGeom.FaceAlpha = 0.5;
        Linkage.EnvironmentBodies(ie).hGeom.FaceColor = [0.7 0.7 0.7];

        set(Linkage.EnvironmentBodies(ie).hGeom, ...
            'FaceLighting', 'gouraud', ...
            'SpecularStrength', 0.0, ...
            'SpecularExponent', 5, ...
            'AmbientStrength',  0.6, ...
            'DiffuseStrength',  0.7);
    end

    axis tight;
    view(3);

    %% ---------- Rendering / transparency settings ----------
    lighting gouraud; material dull; 
    set(gcf,'Renderer','opengl');
    set(gcf,'GraphicsSmoothing','on');   % optional
    set(gca,'SortMethod','depth');       % important with transparency

    %% ---------- Checker planes sizing (based on current axis) ----------
    ax2   = axis;  % [xmin xmax ymin ymax zmin zmax]
    R     = max([diff(ax2(1:2)), diff(ax2(3:4)), diff(ax2(5:6))]);
    S     = 2*R;     % half-extent in plane coords
    pitch = R/20;    % checker tile size

    cDark  = [0.15 0.25 0.45];
    cLight = [0.25 0.45 0.75];

    %% ---------- Bottom plane (z = -3) ----------
    draw_checker_plane([0 0 -3], [0 0  1], S, pitch, cDark, cLight);

    %% ---------- Top plane (z = +3) ----------
    draw_checker_plane([0 0  3], [0 0 -1], S, pitch, cDark, cLight);

    axis([PlotParameters.XLim PlotParameters.YLim PlotParameters.ZLim]);
    drawnow;

    for ib = 1:Linkage.nb
        if isgraphics(Linkage.RobotBodies(ib).hGeom)
            Linkage.RobotBodies(ib).hGeom = [];
        end
    end
    
    for ie = 1:Linkage.ne
        if isgraphics(Linkage.EnvironmentBodies(ie).hGeom)
            Linkage.EnvironmentBodies(ie).hGeom = [];
        end
    end


end

%% =======================================================================

function draw_checker_plane(p0, n, S, pitch, cDark, cLight)
% Draw a checkerboard plane patch centered at p0 with normal n.

    n = n / norm(n);

    % Orthonormal basis {u,v} spanning the plane
    if abs(n(1)) < 0.9
        tmp = [1 0 0];
    else
        tmp = [0 1 0];
    end
    u = cross(n, tmp);  u = u / max(norm(u), eps);
    v = cross(n, u);    v = v / max(norm(v), eps);

    % Parametric grid in plane coords
    s  = linspace(-S, S, 81);
    tp = linspace(-S, S, 81);
    [Sg, Tg] = meshgrid(s, tp);

    % World coordinates
    X = p0(1) + Sg.*u(1) + Tg.*v(1);
    Y = p0(2) + Sg.*u(2) + Tg.*v(2);
    Z = p0(3) + Sg.*u(3) + Tg.*v(3);

    % Checker pattern in plane coords
    chk = mod(round(Sg/pitch) + round(Tg/pitch), 2);
    C = zeros([size(chk), 3]);
    for k = 1:3
        C(:,:,k) = cDark(k).*(chk==0) + cLight(k).*(chk==1);
    end

    % Draw
    srf = surf(X, Y, Z, C, 'EdgeColor','none', 'FaceAlpha',0.5);
    srf.SpecularStrength = 0.9;
    srf.DiffuseStrength  = 0.7;
    srf.AmbientStrength  = 0.5;

end
