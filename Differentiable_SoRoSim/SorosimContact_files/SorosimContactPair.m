classdef SorosimContactPair < handle
    properties
        id (1,1) double = NaN

        body1 SorosimContactBody
        body2 SorosimContactBody

        enabled (1,1) logical = true   % user enable/disable this pair
        contact_active (1,1) logical = false % make true if narrow-phase says alpha<1
        alpha_tol (1,1) double = 1e-5   % contact activation tolerance
        
        S struct = struct() % scaled solver data, contains P, bounds1, bounds2

        % Warm start / guess
        guess struct = struct()            % guess.x, guess.alpha, guess.lambda1, guess.lambda2, empty means "no warm start"
        use_warmstart (1,1) logical = true % policy to use a previous solution as warmstart if available
        warmstart (1,1) logical = false    % warmstart available?

        % Broad-phase state
        broadphase_active (1,1) logical = true

        % Solver options (pair-owned)
        newton_opt struct = struct('L',1,'max_iters',30,'tol',1e-10,'verbose',false);
        surrogate_opt struct = struct('fS_values',[1 3 5 7],'enable_scaling',true,'scale_mode','maxRout');

    end

    methods
        function obj = SorosimContactPair(body1, body2, id)
            if nargin >= 1, obj.body1 = body1; end
            if nargin >= 2, obj.body2 = body2; end
            if nargin >= 3, obj.id = id; end

            obj.refreshGeometryCache();
        end

        function refreshGeometryCache(obj)        
            % --- assemble cached S ---

            P = struct();
            P.g1 = eye(4);
            P.g2 = eye(4);  % updated per solve
            P.shape_id1 = obj.body1.shape_id;
            P.shape_id2 = obj.body2.shape_id;
            P.params1   = obj.body1.params;
            P.params2   = obj.body2.params;
        
            obj.S = struct();
            obj.S.P       = P;
            obj.S.bounds1 = obj.body1.bounds;
            obj.S.bounds2 = obj.body2.bounds;
        
            % --- reset solver state ---
            obj.guess = struct();
            obj.warmstart = false;
            obj.contact_active = false;
        end

        function g_12 = get_relative(obj, g1, g2)
            g1C = g1 * obj.body1.g_JC;
            g2C = g2 * obj.body2.g_JC;
            g_12 = ginv(g1C)*g2C;
        end

        function tf = broadphase(obj, g1, g2)
            % Very cheap broad-phase using bounding spheres from Rout.
            % Returns true if worth running iDCOL narrow-phase.

            if ~obj.enabled
                obj.broadphase_active = false;
                tf = false;
                return;
            end

            bounds1 = obj.body1.bounds;
            bounds2 = obj.body2.bounds;

            g_12 = obj.get_relative(g1, g2);

            r = g_12(1:3,4);
            R = bounds1.Rout + bounds2.Rout;
            tf = (r.'*r <= R*R);

            obj.broadphase_active = tf;
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
                out.x = [NaN NaN NaN];
                out.alpha = NaN;
                out.lambda1 = NaN;
                out.lambda2 = NaN;
                success = true;
                return;
            end
        
            % --- relative center to center transform ---
            obj.S.P.g2 = obj.get_relative(g1, g2);
            
            if obj.use_warmstart && obj.warmstart && ~isempty(fieldnames(obj.guess))
                guess_value = obj.guess;
            else
                guess_value = [];
            end

            % call solver
            out = idcol_solve_mex(obj.S, guess_value, obj.newton_opt, obj.surrogate_opt);
        
            % --- interpret result ---
            success = out.converged;
        
            if success
                obj.contact_active = (out.alpha <= 1 + obj.alpha_tol);
                if obj.use_warmstart
                    obj.guess.x = out.x;
                    obj.guess.alpha = out.alpha;
                    obj.guess.lambda1 = out.lambda1;
                    obj.guess.lambda2 = out.lambda2;
                    obj.warmstart = true;
                else
                    obj.warmstart = false;
                    obj.guess = struct();
                end
            else
                obj.warmstart = false;
            end

        end

    end
end
