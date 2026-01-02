function [V, F] = idcol_body_mesh(shape_id, params, bounds, mesh_opt)
%IDCOL_BODY_MESH  Build triangle mesh (V,F) for implicit surface phi(y)=0
%
% Inputs:
%   shape_id : scalar (kept for phi evaluation)
%   params   : column vector
%   bounds   : struct with field .Rout (REQUIRED)
%   mesh_opt : struct with fields (optional)
%       .ng     : grid resolution per axis (default 120)
%       .margin : fractional padding around bbox (default 0.25)
%       .iso    : iso level (default 0)
%
% Outputs:
%   V : Nv x 3 vertices (local frame)
%   F : Nf x 3 faces (triangles)

    % ---- checks ----
    if nargin < 3 || isempty(bounds) || ~isfield(bounds,'Rout')
        error('idcol_body_mesh: bounds.Rout must be provided.');
    end
    if ~isfinite(bounds.Rout) || bounds.Rout <= 0
        error('idcol_body_mesh: bounds.Rout must be positive and finite.');
    end

    if nargin < 4 || isempty(mesh_opt), mesh_opt = struct(); end
    if ~isfield(mesh_opt,'ng'),     mesh_opt.ng = 120; end
    if ~isfield(mesh_opt,'margin'), mesh_opt.margin = 0.25; end
    if ~isfield(mesh_opt,'iso'),    mesh_opt.iso = 0; end

    params = params(:);
    sid    = double(shape_id);

    % ---- 1) bounding box from Rout ----
    s = (1 + mesh_opt.margin) * bounds.Rout;
    minXYZ = [-s -s -s];
    maxXYZ = [ s  s  s];

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

    if isempty(V) || isempty(F)
        error('idcol_body_mesh: empty isosurface. Increase mesh_opt.margin or check Rout.');
    end
end

%% HELPER
function phi = idcol_mex_phi_only(Y, shape_id, params)
%Y is 3xN.
%Returns 1xN phi values.

    N = size(Y,2);
    phi = zeros(1,N);

    try
        out = shape_core_mex('local_phi_grad', Y, shape_id, params);

        if isnumeric(out) && size(out,2) == N
            phi = out(1,:);
            return;
        end

        if isstruct(out) && isfield(out,'phi')
            phi = reshape(out.phi, 1, []);
            return;
        end
    catch
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
