function params = idcol_scale_params(shape_id, params, s)
% Scale iDCOL shape params by length factor s (typically s = 1/L).
% Only length-like entries are scaled.

switch double(shape_id)

    case 1  % Sphere: [R]
        params(1) = s*params(1);

    case 2  % Polytope: [beta; m; Lscale; A(:); b]
        m = round(params(2));
        params(3) = s*params(3);         % Lscale
        b0 = 3 + 3*m + 1;                % start index of b
        b1 = 3 + 4*m;                    % end index of b
        params(b0:b1) = s*params(b0:b1); % b

    case 3  % Superellipsoid: [n; a; b; c]
        params(2:4) = s*params(2:4);

    case 4  % Superelliptic cylinder: [n; R; h]
        params(2:3) = s*params(2:3);

    case 5  % Truncated cone: [beta; Rb; Rt; a; b]
        params(2:5) = s*params(2:5);

end

end
