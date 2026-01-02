function params = idcol_make_params(shape_id, varargin)
%IDC0L_MAKE_PARAMS Create iDCOL params vector for a given shape_id.
%
% Outputs:
%   params : column vector packed exactly as expected by your iDCOL evaluators
%
% Supported shapes (shape_id):
%   1 Sphere:                 params = [R]
%       call: idcol_make_params(1, R)
%
%   2 Polytope (smooth-max):  params = [beta; m; Lscale; A(:); b]
%       call: idcol_make_params(2, beta, A, b, Lscale)
%       where A is m-by-3, b is m-by-1, Lscale optional (default computed)
%
%   3 Superellipsoid:         params = [n; a; b; c]
%       call: idcol_make_params(3, n, a, b, c)
%
%   4 Superelliptic cylinder: params = [n; R; h]
%       call: idcol_make_params(4, n, R, h)
%
%   5 Truncated cone:         params = [beta; Rb; Rt; a; b]
%       call: idcol_make_params(5, beta, Rb, Rt, a, b)

sid = double(shape_id);

switch sid
    case 1
        R = varargin{1};
        params = R;

    case 2
        beta = varargin{1};
        A    = varargin{2};
        b    = varargin{3};
    
        A = reshape(A, [], 3);
        b = b(:);
        m = size(A,1);
    
        % --- normalize halfspaces: make each row of A a unit normal ---
        rowNorm = sqrt(sum(A.^2, 2));              % m×1
    
        A = A ./ rowNorm;                          % each row unit length
        b = b ./ rowNorm;                          % keep Ax <= b equivalent
    
        % --- optional Lscale (after normalization) ---
        if numel(varargin) >= 4 && ~isempty(varargin{4})
            Lscale = varargin{4};
        else
            % typical magnitude of offsets
            Lscale = max(1, max(abs(b)));
        end
    
        % pack: [beta; m; Lscale; A(:); b]
        params = [beta; m; Lscale; A(:); b];

    case 3
        n = varargin{1};
        a = varargin{2};
        b = varargin{3};
        c = varargin{4};
        params = [n; a; b; c];

    case 4
        n = varargin{1};
        R = varargin{2};
        h = varargin{3};
        params = [n; R; h];

    case 5
        beta = varargin{1};
        Rb   = varargin{2};
        Rt   = varargin{3};
        a    = varargin{4};
        b   = varargin{5};
        params = [beta; Rb; Rt; a; b];

    otherwise
        error('Unknown shape_id: %g', sid);
end

params = params(:); % always return column
end
