function drawFrame(T, options)
    % T: 4x4 Homogeneous Transformation Matrix
    %
    % Example:
    %   drawFrame(T)                         % opaque RGB axes
    %   drawFrame(T, alpha=0.2)              % transparent axes
    %   drawFrame(T, color=[1 0 0; 0 1 0; 0 0 1], alpha=0.5)

    arguments
        T (4,4) double
        % color: either 'rgb' or 3x3 numeric [cx; cy; cz]
        options.color = 'rgb'
        options.framesize (1,1) double = 0.1
        options.linewidth (1,1) double = 1.5
        options.style = "-"   % kept for compatibility (ignored for patches)
        options.alpha (1,1) double {mustBeGreaterThanOrEqual(options.alpha,0), ...
                                    mustBeLessThanOrEqual(options.alpha,1)} = 1
    end

    % Extract rotation matrix and origin
    R      = T(1:3, 1:3);
    origin = T(1:3, 4);
    L      = options.framesize;

    % Resolve colors for X, Y, Z
    [cX, cY, cZ] = resolveFrameColors(options.color);

    hold on

    % Draw X, Y, Z axes as cones
    drawArrowCone(origin, R(:,1)*L, cX, options.alpha);
    drawArrowCone(origin, R(:,2)*L, cY, options.alpha);
    drawArrowCone(origin, R(:,3)*L, cZ, options.alpha);

    % Optional: make it look nicer
    axis equal
end

% ---------- helper: resolve 'rgb' or numeric color ----------
function [cX, cY, cZ] = resolveFrameColors(colorOpt)
    if ischar(colorOpt) || isstring(colorOpt)
        s = char(colorOpt);
        if numel(s) ~= 3
            error('If color is char, it must be like ''rgb'' or ''cmy'', length 3.');
        end
        cmap = struct('r',[1 0 0],'g',[0 1 0],'b',[0 0 1], ...
                      'c',[0 1 1],'m',[1 0 1],'y',[1 1 0],'k',[0 0 0],'w',[1 1 1]);
        cX = cmap.(s(1));
        cY = cmap.(s(2));
        cZ = cmap.(s(3));
    else
        % numeric 3x3: [cx; cy; cz]
        if ~isequal(size(colorOpt), [3 3])
            error('Numeric color must be 3x3: [cx; cy; cz].');
        end
        cX = colorOpt(1,:);
        cY = colorOpt(2,:);
        cZ = colorOpt(3,:);
    end
end

% ---------- helper: draw a cone arrow from p0 in direction vec ----------
function drawArrowCone(p0, vec, color, alpha)
    % p0: 3x1 origin
    % vec: 3x1 direction * length
    % color: 1x3 RGB
    % alpha: scalar in [0,1]

    p0 = p0(:);
    vec = vec(:);
    L = norm(vec);
    if L < 1e-9
        return;
    end
    dir = vec / L;

    % simple geometry: shaft + head as two cones (or one cone if you like)
    shaftFrac = 0.75;
    shaftLen  = shaftFrac * L;
    headLen   = (1 - shaftFrac) * L;

    r_shaft = 0.02 * L;
    r_head  = 0.04 * L;

    nFaces = 16;

    % Build a unit cone along +Z, then transform
    [Xs, Ys, Zs] = cylinder([r_shaft r_shaft], nFaces);  % shaft cylinder
    Zs = Zs * shaftLen;

    [Xh, Yh, Zh] = cylinder([r_head 0], nFaces);         % head cone
    Zh = shaftLen + Zh * headLen;

    % Rotation: align [0;0;1] to dir
    zAxis = [0;0;1];
    Ralign = rotz_to_vec(zAxis, dir);   % 3x3

    % shaft
    Ps = [Xs(:), Ys(:), Zs(:)]';
    Ps_rot = Ralign * Ps;
    Ps_rot = Ps_rot + p0;
    Xs2 = reshape(Ps_rot(1,:), size(Xs));
    Ys2 = reshape(Ps_rot(2,:), size(Ys));
    Zs2 = reshape(Ps_rot(3,:), size(Zs));

    % head
    Ph = [Xh(:), Yh(:), Zh(:)]';
    Ph_rot = Ralign * Ph;
    Ph_rot = Ph_rot + p0;
    Xh2 = reshape(Ph_rot(1,:), size(Xh));
    Yh2 = reshape(Ph_rot(2,:), size(Yh));
    Zh2 = reshape(Ph_rot(3,:), size(Zh));

    % Draw shaft
    surf(Xs2, Ys2, Zs2, ...
        'FaceColor', color, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', alpha);

    % Draw head
    surf(Xh2, Yh2, Zh2, ...
        'FaceColor', color, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', alpha);
end

% ---------- helper: rotation matrix that maps v1 to v2 ----------
function R = rotz_to_vec(v1, v2)
    v1 = v1(:)/norm(v1);
    v2 = v2(:)/norm(v2);

    c = dot(v1,v2);
    if c > 1-1e-12
        R = eye(3);
        return;
    elseif c < -1+1e-12
        % 180-degree rotation about any axis perpendicular to v1
        % pick an arbitrary perpendicular
        if abs(v1(1)) < 0.9
            a = cross(v1, [1;0;0]);
        else
            a = cross(v1, [0;1;0]);
        end
        a = a/norm(a);
        R = axang2rotm_local([a' pi]);
        return;
    end

    a = cross(v1,v2);
    s = norm(a);
    a = a/s;
    K = [   0   -a(3)  a(2);
          a(3)    0   -a(1);
         -a(2)  a(1)    0   ];
    R = eye(3) + K*s + K*K*(1-c);
end

function R = axang2rotm_local(axang)
    % Minimal axang2rotm implementation; axang = [ax_x ax_y ax_z theta]
    a = axang(1:3);
    theta = axang(4);
    a = a(:)/norm(a);
    K = [   0   -a(3)  a(2);
          a(3)    0   -a(1);
         -a(2)  a(1)    0   ];
    c = cos(theta);
    R = eye(3) + sin(theta)*K + (1-c)*(K*K);
end
