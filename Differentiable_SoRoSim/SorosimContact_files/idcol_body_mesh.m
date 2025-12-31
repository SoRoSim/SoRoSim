function [V, F] = idcol_body_mesh(shape_id, params, mesh_opt)
    %IDCOL_BODY_MESH  Build triangle mesh (V,F) for implicit surface phi(y)=0
    % using shape_core_mex("local_phi_grad", ...).
    %
    % Inputs:
    %   shape_id : scalar (1..5)
    %   params   : column vector
    %   mesh_opt : struct with fields (optional)
    %       .ng     : grid resolution per axis (default 120)
    %       .margin : fractional padding around bbox (default 0.25)
    %       .iso    : iso level (default 0)
    %
    % Outputs:
    %   V : Nv x 3 vertices (local frame)
    %   F : Nf x 3 faces (triangles)
    
    if nargin < 3 || isempty(mesh_opt), mesh_opt = struct(); end
    if ~isfield(mesh_opt,'ng'),     mesh_opt.ng = 120; end
    if ~isfield(mesh_opt,'margin'), mesh_opt.margin = 0.25; end
    if ~isfield(mesh_opt,'iso'),    mesh_opt.iso = 0; end
    
    params = params(:);
    sid = double(shape_id);
    
    % ---- 1) pick a bbox that contains the surface ----
    [minXYZ, maxXYZ] = idcol_bbox_guess(sid, params, mesh_opt.margin);
    
    % ---- 2) sample phi on a grid ----
    ng = mesh_opt.ng;
    [xg, yg, zg] = meshgrid( ...
        linspace(minXYZ(1), maxXYZ(1), ng), ...
        linspace(minXYZ(2), maxXYZ(2), ng), ...
        linspace(minXYZ(3), maxXYZ(3), ng));
    
    Y = [xg(:)'; yg(:)'; zg(:)'];  % 3 x N
    
    phi = idcol_mex_phi_only(Y, sid, params);
    phi_grid = reshape(phi, size(xg));
    
    % ---- 3) extract iso-surface ----
    fv = isosurface(xg, yg, zg, phi_grid, mesh_opt.iso);
    
    V = fv.vertices;
    F = fv.faces;
    
    % Safety: some shapes / bboxes might yield empty iso (bad bbox or wrong params)
    if isempty(V) || isempty(F)
        error('idcol_body_mesh: empty isosurface. Increase bbox margin or check params.');
    end
end

%% HELPER 1
function [minXYZ, maxXYZ] = idcol_bbox_guess(shape_id, p, margin)
    % Dimensionless margin around an estimated local bounding box.
    
    p = p(:);
    
    switch double(shape_id)
    
        case 1  % sphere: [R]
            R = p(1);
            s = (1+margin)*R;
            minXYZ = [-s -s -s];
            maxXYZ = [ s  s  s];
    
        case 3  % superellipsoid: [n; a; b; c]
            a = p(2); b = p(3); c = p(4);
            minXYZ = -(1+margin)*[a b c];
            maxXYZ =  (1+margin)*[a b c];
    
        case 4  % superelliptic cylinder: [n; R; h] axis = x, x in [-h,h]
            R = p(2); h = p(3);
            minXYZ = [-(1+margin)*h, -(1+margin)*R, -(1+margin)*R];
            maxXYZ = [ (1+margin)*h,  (1+margin)*R,  (1+margin)*R];
    
        case 5  % truncated cone: [beta; Rb; Rt; a; b], x in [-a, +b]
            Rb = p(2); Rt = p(3); a = p(4); b = p(5);
            R = max(Rb, Rt);
            minXYZ = [-(1+margin)*a, -(1+margin)*R, -(1+margin)*R];
            maxXYZ = [ (1+margin)*b,  (1+margin)*R,  (1+margin)*R];
    
        case 2  % polytope: [beta; m; Lscale; A(:); b]
            m = round(p(2));
            A = reshape(p(4:3+3*m), [m,3]);
            bvec = p(3+3*m+1 : 3+4*m);
    
            % Best bbox: compute vertices if you have halfspace_vertices_3d
            V = [];
            if exist('halfspace_vertices_3d','file') == 2
                V = halfspace_vertices_3d(A, bvec, 1e-10, 1e-9);
            end
    
            if ~isempty(V)
                mn = min(V,[],1);
                mx = max(V,[],1);
                c  = 0.5*(mn+mx);
                r  = 0.5*(mx-mn);
                minXYZ = c - (1+margin)*r;
                maxXYZ = c + (1+margin)*r;
            else
                % Fallback: rough scale from b
                R = max(abs(bvec));
                if ~isfinite(R) || R <= 0, R = 1; end
                s = (1+margin)*R;
                minXYZ = [-s -s -s];
                maxXYZ = [ s  s  s];
            end
    
        otherwise
            minXYZ = [-1 -1 -1];
            maxXYZ = [ 1  1  1];
    end
end

%% HELPER 2
function phi = idcol_mex_phi_only(Y, shape_id, params)
    %Y is 3xN.
    %Returns 1xN phi values.
    
    N = size(Y,2);
    phi = zeros(1,N);
    
    % Try batched call first
    try
        out = shape_core_mex('local_phi_grad', Y, shape_id, params);
    
        % If your mex returns [phi; grad] stacked:
        %   out is (1+3)xN, phi is first row.
        if isnumeric(out) && size(out,2) == N
            phi = out(1,:);
            return;
        end
    
        % If your mex returns struct with field 'phi'
        if isstruct(out) && isfield(out,'phi')
            phi = reshape(out.phi, 1, []);
            return;
        end
    
        % Otherwise fall back
    catch
        % fall back to pointwise
    end
    
    for k = 1:N
        outk = shape_core_mex('local_phi_grad', Y(:,k), shape_id, params);
        if isnumeric(outk)
            phi(k) = outk(1);
        elseif isstruct(outk) && isfield(outk,'phi')
            phi(k) = outk.phi;
        else
            error('idcol_mex_phi_only: unsupported mex output format.');
        end
    end
end
