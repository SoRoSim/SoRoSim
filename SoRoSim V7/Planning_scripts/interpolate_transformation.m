function g = interpolate_transformation(g_init,g_final, number_of_interpolation_points)
%%% between two poses in SE(3), it finds multiple intermidiate poses along
%%% the screw
    % g_init = [0.0000         0   -1.0000    0.3
    %      0    1.0000         0         0
    % 1.0000         0    0.0000    0.438
    %      0         0         0    1.0000]; %% This is used to get the initial guess
    V = piecewise_logmap(ginv(g_init)*g_final);
    s = 0:1/number_of_interpolation_points:1;
    g = zeros(4*number_of_interpolation_points, 4);
    % plotTransforms(se3(g_init),'FrameSize',0.1);
    % hold on
    for i =1:number_of_interpolation_points
        g(4*i-3:4*i,1:4) = g_init*variable_expmap_g(V*s(i));
        % plotTransforms(se3(g(4*i-3:4*i,1:4)),'FrameSize',0.03)
    end
    % plotTransforms(se3(g_final), 'FrameSize',0.1)
    g = [g;g_final];
end