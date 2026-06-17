function plotqt_drone(Linkage, P, fps, outFile)
% plotqt_temp  Render a translation-only trajectory and save as MP4,
% also drawing the traversed path progressively.
%
%   plotqt_temp(Linkage, P)
%   plotqt_temp(Linkage, P, fps)
%   plotqt_temp(Linkage, P, fps, outFile)
%
% Inputs
%   Linkage  : SoRoSim linkage
%   P        : 3xN knot positions (translation only)
%   fps      : frames per second (default: 30)
%   outFile  : output mp4 filename (default: 'trajectory.mp4')

    if nargin < 3 || isempty(fps), fps = 30; end
    if nargin < 4 || isempty(outFile), outFile = 'trajectory.mp4'; end

    validateattributes(P, {'double','single'}, {'2d','nrows',3,'finite','real'});
    N = size(P,2);

    PlotParameters = Linkage.PlotParameters;

    %% ---------- Figure / axes ----------
    fh = figure;
    ax = axes('Parent', fh);
    set(fh,'Units','pixels');
    set(fh,'Position',[50 50 1280 720]);   % both even -> no padding warning


    set(ax, ...
        'CameraPosition', PlotParameters.CameraPosition, ...
        'CameraTarget',   PlotParameters.CameraTarget, ...
        'CameraUpVector', PlotParameters.CameraUpVector, ...
        'FontSize', 18);

    if isfield(PlotParameters,'Light') && PlotParameters.Light
        camlight(ax, PlotParameters.Az_light, PlotParameters.El_light);
    end

    axis(ax,'equal'); grid(ax,'on'); hold(ax,'on');
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');

    set(get(ax,'Title'),  'Interpreter','latex');
    set(get(ax,'XLabel'), 'Interpreter','latex');
    set(get(ax,'YLabel'), 'Interpreter','latex');
    set(get(ax,'ZLabel'), 'Interpreter','latex');
    set(ax,'TickLabelInterpreter','latex');
    set(ax,'FontSize',18);

    %% ---------- Attach and draw bodies once ----------
    for ib = 1:Linkage.nb
        Linkage.RobotBodies(ib).attachHandles(ax,[],[],false);
        Linkage.RobotBodies(ib).hGeom.FaceColor = [0 1 0];
        Linkage.RobotBodies(ib).hGeom.FaceAlpha = 1;
    end
    for ie = 1:Linkage.ne
        Linkage.EnvironmentBodies(ie).attachHandles(ax,[],[],false);
        Linkage.EnvironmentBodies(ie).setPose(eye(4));
        Linkage.EnvironmentBodies(ie).hGeom.FaceAlpha = 0.5;
        Linkage.EnvironmentBodies(ie).hGeom.FaceColor = [0.7 0.7 0.7];
    end

    axis(ax,'tight'); %view(ax,3);
    lighting gouraud; material dull;
    set(fh,'Renderer','opengl');
    try set(fh,'GraphicsSmoothing','on'); catch, end
    set(ax,'SortMethod','depth');

    %% ---------- Checker planes ----------
    ax2 = axis(ax);
    R   = max([diff(ax2(1:2)), diff(ax2(3:4)), diff(ax2(5:6))]);
    S   = 2*R;
    pitch = R/20;

    cDark  = [0.15 0.25 0.45];
    cLight = [0.25 0.45 0.75];

    addCheckerPlane(ax, [0 0 -3], [0 0  1], S, pitch, cDark, cLight, 0.5);
    addCheckerPlane(ax, [0 0  3], [0 0 -1], S, pitch, cDark, cLight, 0.5);

    if isfield(PlotParameters,'XLim') && isfield(PlotParameters,'YLim') && isfield(PlotParameters,'ZLim')
        axis(ax, [PlotParameters.XLim PlotParameters.YLim PlotParameters.ZLim]);
    end

    %% ---------- Path objects (progressive drawing) ----------
    % full path (faint) + traversed path (solid) + moving point
    h_path_all = plot3(ax, P(1,:), P(2,:), P(3,:), '--', 'LineWidth', 1.0);
    h_path_all.Color(4) = 0.25; % alpha if supported (R2022a+). If not, ignore.

    h_path_prog = plot3(ax, P(1,1), P(2,1), P(3,1), '-', 'LineWidth', 2.0);
    h_pt = plot3(ax, P(1,1), P(2,1), P(3,1), 'o', 'MarkerSize', 6, 'LineWidth', 1.5);

    drawnow;

    %% ---------- Video writer ----------
    vw = VideoWriter(outFile, 'MPEG-4');
    vw.FrameRate = fps;
    open(vw);
    cleanupObj = onCleanup(@() safeCloseVideo(vw));
    p0 = P(:,1);
    pT = P(:,end);
    plot3(p0(1), p0(2), p0(3), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(pT(1), pT(2), pT(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
%% text handles
    ncp = Linkage.ncp;
    
    h_alpha_txt = gobjects(ncp,1);
    for icp = 1:ncp
        h_alpha_txt(icp) = text(ax, 0,0,0, '', ...
            'Interpreter','latex', ...
            'FontSize', 14, ...
            'HorizontalAlignment','center', ...
            'VerticalAlignment','middle');
    end
    %% ---------- Animate ----------
    for k = 1:N
        % update robot pose (translation only)
        g1 = eye(4);
        g1(1:3,4) = P(:,k);
        for ib = 1:Linkage.nb
            Linkage.RobotBodies(ib).setPose(g1);
        end

        for icp = 1:ncp
            g2 = eye(4);
            % -------- solve contact --------
            out = Linkage.Pairs(icp).solveContact(g1, g2);
            p_txt = Linkage.Pairs(icp).body2.g_JC(1:3,4);
            if icp == 5 % a small plotting adjustment for frusta
                p_txt(3) = 1;
            end
            % Show alpha (robustly)
            if isfield(out,'alpha') && ~isempty(out.alpha) && isfinite(out.alpha)
                str = sprintf('$\\alpha^*=%.3f$', out.alpha);
            else
                str = '$\\alpha^*=\mathrm{NaN}$';   % if you prefer, set '' to hide
            end
    
            set(h_alpha_txt(icp), ...
                'Position', p_txt(:).', ...
                'String', str, ...
                'Visible', 'on');

        end

        % update path graphics
        set(h_path_prog, 'XData', P(1,1:k), 'YData', P(2,1:k), 'ZData', P(3,1:k));
        set(h_pt,        'XData', P(1,k),   'YData', P(2,k),   'ZData', P(3,k));

        drawnow;
        writeVideo(vw, getframe(fh));
    end

    safeCloseVideo(vw);
end

%% ======================= helpers =======================

function addCheckerPlane(ax, p0, n, S, pitch, cDark, cLight, alpha)
    n = n(:).';
    n = n / max(norm(n), eps);

    if abs(n(1)) < 0.9, tmp = [1 0 0]; else, tmp = [0 1 0]; end
    u = cross(n, tmp);  u = u / max(norm(u), eps);
    v = cross(n, u);    v = v / max(norm(v), eps);

    s  = linspace(-S, S, 81);
    tp = linspace(-S, S, 81);
    [Sg, Tg] = meshgrid(s, tp);

    X = p0(1) + Sg.*u(1) + Tg.*v(1);
    Y = p0(2) + Sg.*u(2) + Tg.*v(2);
    Z = p0(3) + Sg.*u(3) + Tg.*v(3);

    chk = mod(round(Sg/pitch) + round(Tg/pitch), 2);
    C = zeros([size(chk), 3]);
    for kk = 1:3
        C(:,:,kk) = cDark(kk).*(chk==0) + cLight(kk).*(chk==1);
    end

    srf = surf(ax, X, Y, Z, C, 'EdgeColor','none', 'FaceAlpha', alpha);
    srf.SpecularStrength = 0.9;
    srf.DiffuseStrength  = 0.7;
    srf.AmbientStrength  = 0.5;
end

function safeCloseVideo(vw)
    if ~isempty(vw) && isvalid(vw)
        try close(vw); catch, end
    end
end
