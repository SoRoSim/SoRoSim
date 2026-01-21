function P = resample_polyline(pts, N)
% pts: 3xM
M = size(pts,2);
if M < 2
    P = repmat(pts,1,N);
    return;
end

seg = sqrt(sum(diff(pts,1,2).^2,1));     % 1x(M-1)
s = [0, cumsum(seg)];
L = s(end);

s_query = linspace(0, L, N);
P = zeros(3,N);

j = 1;
for k = 1:N
    tq = s_query(k);
    while (j < M-1) && (s(j+1) < tq)
        j = j + 1;
    end
    if s(j+1) == s(j)
        alpha = 0;
    else
        alpha = (tq - s(j)) / (s(j+1) - s(j));
    end
    P(:,k) = (1-alpha)*pts(:,j) + alpha*pts(:,j+1);
end
end
