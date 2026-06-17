function T = sample_state(joint_angle
% randSE3  Sample a random SE(3) pose within position & Euler angle bounds.
%
% pos_bounds = [xmin xmax; ymin ymax; zmin zmax];
% eul_bounds = [rmin rmax; pmin pmax; ymin ymax];  % roll, pitch, yaw bounds (rad)
%
% Output:
%   T = 4x4 homogeneous transform
    T = zeros(8,4);
    for i=1:2
        % --- Random position ---
        px = pos_bounds(1,1) + (pos_bounds(1,2) - pos_bounds(1,1)) * rand;
        py = pos_bounds(2,1) + (pos_bounds(2,2) - pos_bounds(2,1)) * rand;
        pz = pos_bounds(3,1) + (pos_bounds(3,2) - pos_bounds(3,1)) * rand;
        p  = [px; py; pz];
    
        % --- Random Euler angles ---
        roll  = eul_bounds(1,1) + (eul_bounds(1,2) - eul_bounds(1,1)) * rand;
        pitch = eul_bounds(2,1) + (eul_bounds(2,2) - eul_bounds(2,1)) * rand;
        yaw   = eul_bounds(3,1) + (eul_bounds(3,2) - eul_bounds(3,1)) * rand;
    
        % MATLAB: ZYX Euler order -> rotm
        R = eul2rotm([yaw pitch roll]);
    
        % --- Output SE(3) homogeneous matrix ---
        T(4*i-3:4*i,:) = [R, p; 0 0 0 1];
    end
end
