function [t, g] = generate_trajectory(shape, T_final,g_0)
    t = 0:0.1:T_final;
    if shape == "xz_circle"
        r = 1;
        w = 2*pi/T_final;
        x = r*sin(w*t);
        z = r -r*cos(w*t);

        g = repmat(g_0,length(t),1);
        g(1:4:end,4) = g(1:4:end,4)+x';
        g(3:4:end,4) = g(3:4:end,4)+z';
    elseif shape == "z_spiral"
        r = 1;
        w = 2*2*pi/T_final; % two rotations
        v = 0.1;
        x = r*sin(w*t);
        y = r - r*cos(w*t);
        z = v*t;
        g = repmat(g_0,length(t),1);
        g(1:4:end,4) = g(1:4:end,4)+x';
        g(2:4:end,4) = g(2:4:end,4)+y';
        g(3:4:end,4) = g(3:4:end,4)+z';
    elseif shape == "rotation_x"
        r = 1;
        w = 2*pi/T_final;
        g = repmat(g_0,length(t),1);
        for i = 1:length(t)
            g_rot = [1 0 0 0
                0 cos(w*t(i)) -sin(w*t(i)) 0
                0 sin(w*t(i)) cos(w*t(i)) 0
                0 0 0 1];
            g(4*i-3:4*i,1:4) = g(4*i-3:4*i, 1:4)*g_rot;
        end

    elseif shape == "rotation_y"
        w = 2*pi/T_final;
        g = repmat(g_0,length(t),1);
        g(1:4:end, 1) = cos(w*t);
        g(3:4:end, 3) = cos(w*t);
        g(3:4:end, 1) = -sin(w*t);
        g(1:4:end, 3) = sin(w*t);

    elseif shape == "rotation_z"
        w = 2*pi/T_final;
        g = repmat(g_0, length(t), 1);
        g(1:4:end, 1) = cos(w*t);
        g(2:4:end, 2) = cos(w*t);
        g(1:4:end, 2) = -sin(w*t);
        g(2:4:end, 1) = sin(w*t);
    elseif shape == "z_spiral with rotation_x"
        w = 2*pi/T_final;
        g = repmat(g_0, length(t), 1);
        r = 1;
        v = 0.1;
        for i = 1:length(t)
            g_rot = [1 0 0 r*sin(w*t(i))
                0 cos(w*t(i)) -sin(w*t(i)) r-r*cos(w*t(i))
                0 sin(w*t(i)) cos(w*t(i)) v*t(i)
                0 0 0 1];
            g(4*i-3:4*i,1:4) = g(4*i-3:4*i, 1:4)*g_rot;
        end
    end
    