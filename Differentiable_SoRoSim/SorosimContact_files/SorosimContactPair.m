classdef SorosimContactPair < handle
    properties
        id (1,1) double = NaN

        body1 SorosimContactBody
        body2 SorosimContactBody

        enabled (1,1) logical = true   % user enable/disable this pair
        contact_active (1,1) logical = false % make true if narrow-phase says alpha<1

        % Scaling
        L (1,1) double = 1.0
        invL (1,1) double = 1.0
        
        S struct = struct() % scaled solver data, contains P, bounds1, bounds2 (in scaled space)

        % Warm start / guess
        guess struct = struct()         % guess.x, guess.alpha, guess.lambda1, guess.lambda2, empty means "no warm start"
        warmstart (1,1) logical = false % make true when needed

        % Broad-phase state
        broadphase_active (1,1) logical = true

        % Options
        scale_mode char = 'maxRout'   % 'maxRout' or 'sumRout'

        % Solver options (pair-owned)
        newton_opt struct = struct('L',1,'max_iters',30,'tol',1e-10,'verbose',false);
        surrogate_opt struct = struct('fS_values',[1 3 5 7]);

    end

    methods
        function obj = SorosimContactPair(body1, body2, id)
            if nargin >= 1, obj.body1 = body1; end
            if nargin >= 2, obj.body2 = body2; end
            if nargin >= 3, obj.id = id; end

            obj.refreshGeometryCache();
        end

        function refreshGeometryCache(obj)

            % --- real bounds (for scaling + broadphase) ---
            bounds1 = obj.body1.getBounds();
            bounds2 = obj.body2.getBounds();
        
            % --- choose scale ---
            switch obj.scale_mode
                case 'maxRout'
                    obj.L = max(bounds1.Rout, bounds2.Rout);
                case 'sumRout'
                    obj.L = bounds1.Rout + bounds2.Rout;
                otherwise
                    obj.L = max(bounds1.Rout, bounds2.Rout);
            end
            if obj.L <= 0, obj.L = 1; end
            obj.invL = 1 / obj.L;
        
            % --- scale params ---
            params1_s = idcol_scale_params(obj.body1.shape_id, obj.body1.params, obj.invL);
            params2_s = idcol_scale_params(obj.body2.shape_id, obj.body2.params, obj.invL);
        
            % --- scale bounds ---
            bounds1_s = bounds1;
            bounds2_s = bounds2;
        
            if isfield(bounds1_s,'Rout'), bounds1_s.Rout = obj.invL * bounds1.Rout; end
            if isfield(bounds2_s,'Rout'), bounds2_s.Rout = obj.invL * bounds2.Rout; end
            if isfield(bounds1_s,'Rin'),  bounds1_s.Rin  = obj.invL * bounds1.Rin;  end
            if isfield(bounds2_s,'Rin'),  bounds2_s.Rin  = obj.invL * bounds2.Rin;  end
        
            % --- assemble cached S ---
            P = struct();
            P.g1 = eye(4);
            P.g2 = eye(4);  % updated per solve
            P.shape_id1 = obj.body1.shape_id;
            P.shape_id2 = obj.body2.shape_id;
            P.params1   = params1_s;
            P.params2   = params2_s;
        
            obj.S = struct();
            obj.S.P       = P;
            obj.S.bounds1 = bounds1_s;
            obj.S.bounds2 = bounds2_s;
        
            % --- reset solver state ---
            obj.guess = struct();
            obj.warmstart = false;
            obj.contact_active = false;
        end


        function tf = broadphase(obj, g1, g2)
            % Very cheap broad-phase using bounding spheres from Rout.
            % Returns true if worth running iDCOL narrow-phase.

            if ~obj.enabled
                obj.broadphase_active = false;
                tf = false;
                return;
            end

            bounds1 = obj.body1.getBounds();
            bounds2 = obj.body2.getBounds();

            % centers in world (using g_JC offsets)
            g1C = g1 * obj.body1.g_JC;
            g2C = g2 * obj.body2.g_JC;

            c1 = g1C(1:3,4);
            c2 = g2C(1:3,4);

            d = norm(c2 - c1);
            tf = (d <= (bounds1.Rout + bounds2.Rout));

            obj.broadphase_active = tf;
        end

        function g12_s = relativeScaledTransform(obj, g1, g2)
            % Returns scaled relative transform g_12 in body1 contact frame,
            % plus translation r_s.

            g1C = g1 * obj.body1.g_JC;
            g2C = g2 * obj.body2.g_JC;
            g12_s = ginv(g1C) * g2C; %relative transformation
            g12_s(1:3,4) = obj.invL * g12_s(1:3,4); %scaling

        end

        function [out, success] = solveNarrowPhase(obj, g1, g2)

            success = false;
            out = [];
        
            % --- guards ---
            if ~obj.enabled
                return;
            end
        
            if ~obj.broadphase(g1, g2)
                obj.contact_active = false;
                return;
            end
        
            % --- relative scaled transform ---
            obj.S.P.g2 = obj.relativeScaledTransform(g1, g2);
            
            if obj.warmstart && ~isempty(fieldnames(obj.guess))
                guess_value = obj.guess;
            else
                guess_value = [];
            end

            % call solver
            out = idcol_solve_mex(obj.S, guess_value, obj.newton_opt, obj.surrogate_opt);
        
            % --- interpret result ---
            success = out.converged;
        
            if success
                obj.guess = out.guess;     % or however your mex exposes it
                obj.warmstart = true;
                obj.contact_active = (out.alpha < 1);
            else
                obj.warmstart = false;
            end
        end

    end
end
