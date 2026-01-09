classdef SorosimContactBody < handle

    % SorosimContactBody
    % - Stores iDCOL shape description (shape_id, params)
    % - Caches radial bounds via radial_bounds_mex
    % - Caches mesh data (V,F) via idcol_body_mesh (no figure needed)
    % - (Optional) Creates graphics objects (hgtransform + patch) on demand:
    %     set(hT,'Matrix',g)

    properties
        % Identity
        id (1,1) double = NaN % body id, different from Link id
        i_sig (1,1) double = NaN % corresponding significant point

        % iDCOL geometry
        shape_id (1,1) double = 0 % 1: sphere, 2: polytope, 3: superellipsoid, 4: superelliptic cylinder, 5: truncated cone
        params double = []        % column vector preferred

        % Cached bounds
        bounds struct = struct()
        bounds_sig = [] % signature to detect recomputation of bounds

        % Options for bounds computation
        bounds_opt struct = struct('num_starts', 1000)

        % Pose
        g_JC (4,4) double = eye(4) % fixed offset: joint -> contact geometry frame

        % ---- Mesh cache (compute only during plotting, clear after that)----
        meshV double = []   % (Nv x 3) local vertices
        meshF double = []   % (Nf x 3) triangle faces
        mesh_opt struct = struct('ng', 120, 'margin', 0.25) % defaults for idcol_body_mesh
        mesh_sig = []       % signature to detect mesh rebuild

        % ---- Plot handles (optional) ----
        hT = []     % hgtransform
        hGeom = []  % patch/surf
    end

    methods
        function obj = SorosimContactBody(id, shape_id, params)
            % obj = SorosimContactBody(id, shape_id, params)
            if nargin >= 1 && ~isempty(id), obj.id = id; end
            if nargin >= 2 && ~isempty(shape_id), obj.shape_id = shape_id; end
            if nargin >= 3 && ~isempty(params), obj.params = params(:); end

            obj.computeBounds(false);
            %obj.computeMesh([], false); % no need to precompute
        end

        function setGeometry(obj, shape_id, params, recompute_bounds, recompute_mesh)
            % setGeometry(shape_id, params, recompute_bounds=true, recompute_mesh=true)
            if nargin < 4, recompute_bounds = true; end
            if nargin < 5, recompute_mesh   = true; end
        
            obj.shape_id = shape_id;
            obj.params   = params(:);
        
            obj.invalidateBounds();
            obj.invalidateMesh();   % clears mesh + graphics (so visuals won't lie)
        
            if recompute_bounds
                obj.computeBounds(false);
            end
        
            if recompute_mesh
                obj.computeMesh(obj.mesh_opt, false);  % rebuild V,F immediately
            end
        end


        function invalidateBounds(obj)
            obj.bounds = struct();
            obj.bounds_sig = [];
        end

        function computeBounds(obj, force)
            % computeBounds(force=false)
            if nargin < 2, force = false; end

            sig = SorosimContactBody.makeBoundsSignature(obj.shape_id, obj.params, obj.bounds_opt);

            if force || isempty(obj.bounds_sig) || ~isequal(sig, obj.bounds_sig)
                if ~(obj.shape_id==2&&obj.params(2)==1) %not a plane
                    obj.bounds = radial_bounds_mex(obj.shape_id, obj.params, obj.bounds_opt);
                else
                    obj.bounds.Rin2 = 0; obj.bounds.Rout2 = 0; obj.bounds.Rin = 0; obj.bounds.Rout = 0; obj.bounds.xin = NaN(3,1); obj.bounds.xout = NaN(3,1); 
                end
                obj.bounds_sig = sig;
            end
        end

        function b = getBounds(obj)
            if ~isfield(obj.bounds, 'Rout')
                obj.computeBounds(false); %run only if invalid
            end
            b = obj.bounds;
        end

        function setBoundsOptions(obj, bounds_opt, recompute)
            % setBoundsOptions(bounds_opt, recompute=true)
            if nargin < 3, recompute = true; end
            obj.bounds_opt = bounds_opt;
            obj.invalidateBounds();
            if recompute
                obj.computeBounds(false);
            end
        end

        function invalidateMesh(obj)
            obj.meshV = [];
            obj.meshF = [];
            obj.mesh_sig = [];

            % If currently plotted, delete graphics objects too
            if ~isempty(obj.hGeom) && isgraphics(obj.hGeom), delete(obj.hGeom); end
            if ~isempty(obj.hT) && isgraphics(obj.hT), delete(obj.hT); end
            obj.hGeom = [];
            obj.hT = [];
        end

        function setMeshOptions(obj, mesh_opt, recompute)
            % setMeshOptions(mesh_opt, recompute=true)
            if nargin < 3, recompute = true; end
            obj.mesh_opt = mesh_opt;
            obj.invalidateMesh();
            if recompute
                obj.computeMesh(obj.mesh_opt, false);
            end
        end

        function computeMesh(obj, mesh_opt, force)
            % computeMesh(mesh_opt=obj.mesh_opt, force=false)
            if nargin < 2 || isempty(mesh_opt), mesh_opt = obj.mesh_opt; end
            if nargin < 3, force = false; end

            if (obj.shape_id==2&&obj.params(2)==1) %plane
                return; 
            end

            sig = SorosimContactBody.makeMeshSignature(obj.shape_id, obj.params, mesh_opt);

            if force || isempty(obj.mesh_sig) || ~isequal(sig, obj.mesh_sig)

                [V, F] = idcol_body_mesh(obj.shape_id, obj.params, obj.bounds, mesh_opt);

                obj.meshV = V;
                obj.meshF = F;
                obj.mesh_sig = sig;
            end
        end

        function [V, F] = getMesh(obj, mesh_opt)
            % Lazy getter
            if nargin < 2 || isempty(mesh_opt), mesh_opt = obj.mesh_opt; end
            if isempty(obj.meshV) || isempty(obj.meshF)
                obj.computeMesh(mesh_opt, false);
            end
            V = obj.meshV;
            F = obj.meshF;
        end

        function attachHandles(obj, ax, mesh_opt, style)
            % attachHandles(ax, mesh_opt=obj.mesh_opt, style=struct())
            if nargin < 3 || isempty(mesh_opt), mesh_opt = obj.mesh_opt; end
            if nargin < 4, style = struct(); end

            [V, F] = obj.getMesh(mesh_opt);

            obj.invalidateGraphicsOnly();

            obj.hT = hgtransform('Parent', ax);
            obj.hGeom = patch('Parent', obj.hT, 'Faces', F, 'Vertices', V);

            % minimal style defaults
            if ~isfield(style,'EdgeColor'), style.EdgeColor = 'none'; end
            if ~isfield(style,'FaceAlpha'), style.FaceAlpha = 0.35; end

            set(obj.hGeom, 'EdgeColor', style.EdgeColor, 'FaceAlpha', style.FaceAlpha);

            if isfield(style,'FaceColor'), set(obj.hGeom,'FaceColor',style.FaceColor); end
        end

        function ensurePlotHandles(obj, ax, mesh_opt, style)
            % ensurePlotHandles(ax, mesh_opt=obj.mesh_opt, style=struct())
            if nargin < 3 || isempty(mesh_opt), mesh_opt = obj.mesh_opt; end
            if nargin < 4, style = struct(); end

            % Need graphics if missing or invalid
            needG = isempty(obj.hT) || ~isgraphics(obj.hT) || isempty(obj.hGeom) || ~isgraphics(obj.hGeom);
            if needG
                obj.attachHandles(ax, mesh_opt, style);
                return;
            end

            % If mesh params changed since graphics were created, rebuild graphics
            sig = SorosimContactBody.makeMeshSignature(obj.shape_id, obj.params, mesh_opt);
            if isempty(obj.mesh_sig) || ~isequal(sig, obj.mesh_sig)
                obj.invalidateMesh();              % clears mesh + graphics
                obj.attachHandles(ax, mesh_opt, style);
            end
        end

        function grad = get_grad(obj, y)

            out = shape_core_mex('local_phi_grad', y, obj.shape_id, obj.params);
            grad = out.grad;

        end

        function [grad, H] = get_gradH(obj, y)

            out = shape_core_mex('local', y, obj.shape_id, obj.params);
            grad = out.grad;
            H = out.H;

        end

        function setPose(obj, g_here)
            % setPose(g_here): updates hgtransform pose if it exists
            if ~isempty(obj.hT) && isgraphics(obj.hT)
                T = g_here * obj.g_JC;

                % 1) enforce proper homogeneous form
                T(4,:) = [0 0 0 1];
                
                % 2) guard against NaN/Inf
                if any(~isfinite(T(:)))
                    return; % or skip update
                end
                
                % 3) re-orthonormalize rotation (kills numerical drift)
                R = T(1:3,1:3);
                [U,~,V] = svd(R);
                R = U*V.';
                if det(R) < 0
                    U(:,3) = -U(:,3);
                    R = U*V.';
                end
                T(1:3,1:3) = R;
                
                set(obj.hT,'Matrix',T);
            end
        end

        function setJointPose(obj, g_WJ)
            % setJointPose(g_WJ): sets pose using joint pose and fixed offset g_JC
            if ~isempty(obj.hT) && isgraphics(obj.hT)
                set(obj.hT, 'Matrix', g_WJ * obj.g_JC);
            end
        end
    end

    methods(Access=private)
        function invalidateGraphicsOnly(obj)
            % Delete only graphics objects; keep cached mesh data.
            if ~isempty(obj.hGeom) && isgraphics(obj.hGeom), delete(obj.hGeom); end
            if ~isempty(obj.hT) && isgraphics(obj.hT), delete(obj.hT); end
            obj.hGeom = [];
            obj.hT = [];
        end
    end

    methods(Static, Access=private)
        function sig = makeBoundsSignature(shape_id, params, opt)
            p = params(:);
            ns = NaN;
            if isstruct(opt) && isfield(opt,'num_starts')
                ns = opt.num_starts;
            end
            sig = {shape_id, numel(p), sum(p), sum(abs(p)), ns};
        end

        function sig = makeMeshSignature(shape_id, params, mesh_opt)
            p = params(:);
            ng = NaN; margin = NaN;
            if isstruct(mesh_opt)
                if isfield(mesh_opt,'ng'), ng = mesh_opt.ng; end
                if isfield(mesh_opt,'margin'), margin = mesh_opt.margin; end
            end
            sig = {shape_id, numel(p), sum(p), sum(abs(p)), ng, margin};
        end
    end
end
