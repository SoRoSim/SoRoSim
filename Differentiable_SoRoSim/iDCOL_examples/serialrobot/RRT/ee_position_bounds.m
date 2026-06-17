function bounds = ee_position_bounds(Linkage,body_number, q_min, q_max, N)
% Find approximate reachable workspace bounds via random sampling.
%
% bounds: struct with fields x_min, x_max, y_min, y_max, z_min, z_max

    if nargin < 4, N = 50000; end

    xs = zeros(N,1);
    ys = zeros(N,1);
    zs = zeros(N,1);

    for k = 1:N
        q = q_min + rand(size(q_min)) .* (q_max - q_min);
        q_full = zeros(Linkage.ndof,1);
        q_full(1:7) = q(1:7);
        q_full(end-6:end) =q(8:14);
        T = Linkage.FwdKinematics(q_full, body_number(1));  
        p1 = T(5:7,4);

        xs1(k) = p1(1);
        ys1(k) = p1(2);
        zs1(k) = p1(3);

        T = Linkage.FwdKinematics(q_full, body_number(2));  
        p2 = T(5:7,4);

        xs2(k) = p2(1);
        ys2(k) = p2(2);
        zs2(k) = p2(3);

    end

    bounds.x_min1 = min(xs1); bounds.x_max1 = max(xs1);
    bounds.y_min1 = min(ys1); bounds.y_max1 = max(ys1);
    bounds.z_min1 = min(zs1); bounds.z_max1 = max(zs1);
    bounds.x_min2 = min(xs2); bounds.x_max2 = max(xs2);
    bounds.y_min2 = min(ys2); bounds.y_max2 = max(ys2);
    bounds.z_min2 = min(zs2); bounds.z_max2 = max(zs2);
end
