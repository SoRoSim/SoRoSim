function reflect_axes_objects(ax, p0, n, opts) %to be improved
% Fake reflections by mirroring all objects in axes AX across plane (p0,n).
% p0: 1x3 point on plane (world)
% n : 1x3 unit normal of plane (world)
% opts.alpha: reflection alpha   (default 0.12)
% opts.tint : 1x3 RGB multiplier (default [1 1 1])
% opts.copyTag: tag to mark clones (default '__reflection__')

if nargin < 4, opts = struct; end
if ~isfield(opts,'alpha'), opts.alpha = 0.12; end
if ~isfield(opts,'tint'),  opts.tint  = [1 1 1]; end
if ~isfield(opts,'copyTag'), opts.copyTag = '__reflection__'; end

n = n(:)/norm(n);                     % make sure unit
I = eye(3);
R = I - 2*(n*n.');                    % reflection matrix
t = 2*n*(n.'*p0(:));                  % translation term (affine: x' = R*x + t)

kids = all_descendants(ax);
for h = reshape(kids,1,[])

    % skip existing reflections to avoid recursion
    if isgraphics(h) && isfield(get(h),'Tag') && strcmp(get(h,'Tag'),opts.copyTag)
        continue;
    end

    switch lower(h.Type)
        case {'line'}
            X = get(h,'XData'); Y = get(h,'YData'); Z = get(h,'ZData');
            P = [X(:) Y(:) Z(:)]; Pp = (P*R.' + t.').';
            h2 = copyobj(h, ax);
            set(h2,'XData',reshape(Pp(1,:),size(X)), ...
                   'YData',reshape(Pp(2,:),size(Y)), ...
                   'ZData',reshape(Pp(3,:),size(Z)), ...
                   'Tag',opts.copyTag, ...
                   'Color',min(1,opts.tint.*get(h,'Color')), ...
                   'LineStyle',get(h,'LineStyle'), ...
                   'LineWidth',get(h,'LineWidth')*0.9);

        case {'scatter','scatter3'}
            X = get(h,'XData'); Y = get(h,'YData'); Z = get(h,'ZData');
            P = [X(:) Y(:) Z(:)]; Pp = (P*R.' + t.').';
            h2 = copyobj(h, ax);
            set(h2,'XData',reshape(Pp(1,:),size(X)), ...
                   'YData',reshape(Pp(2,:),size(Y)), ...
                   'ZData',reshape(Pp(3,:),size(Z)), ...
                   'Tag',opts.copyTag, ...
                   'MarkerEdgeAlpha',opts.alpha, ...
                   'MarkerFaceAlpha',opts.alpha);

        case {'surface', 'surf'}
            X = get(h,'XData'); Y = get(h,'YData'); Z = get(h,'ZData');
            P = [X(:) Y(:) Z(:)]; Pp = (P*R.' + t.').';
            h2 = copyobj(h, ax);
            set(h2,'XData',reshape(Pp(1,:),size(X)), ...
                   'YData',reshape(Pp(2,:),size(Y)), ...
                   'ZData',reshape(Pp(3,:),size(Z)), ...
                   'Tag',opts.copyTag, ...
                   'FaceAlpha',min(0.5,opts.alpha), ...
                   'EdgeAlpha',0.0);

        case {'patch'}
            V = get(h,'Vertices');      % Nx3
            F = get(h,'Faces');         % Mx3 (or MxK)
        
            Vp = V*R.' + t.';           % reflect vertices
        
            h2 = copyobj(h, ax);
            set(h2, ...
                'Vertices',Vp, ...
                'Faces',fliplr(F), ...  % <<< critical for isosurface
                'Tag',opts.copyTag, ...
                'FaceAlpha',min(0.5,opts.alpha), ...
                'EdgeAlpha',0.0);

        otherwise
            % Ignore UI, lights, texts etc.
    end
end

% draw the mirror plane last (optional): keep your own fancy plane
% Ensure nice lighting
lighting(ax,'phong'); camlight(ax,'headlight'); material(ax,'shiny');
end

function hs = all_descendants(ax)
    hs = ax.Children(:);
    k = 1;
    while k <= numel(hs)
        h = hs(k);
        if isgraphics(h) && isprop(h,'Children') && ~isempty(h.Children)
            hs = [hs; h.Children(:)]; %#ok<AGROW>
        end
        k = k + 1;
    end
end