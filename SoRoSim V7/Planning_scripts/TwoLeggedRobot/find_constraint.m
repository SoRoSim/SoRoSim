% Plotting the rope at q
function [hole_position, roots] = find_constraint(S1,q, constraint_height,leg_index)
    roots = zeros(length(leg_index),1);
    hole_position = zeros(length(leg_index), 3);
    for i =1:length(leg_index)
        g = S1.FwdKinematics(q,leg_index(i));
        Xs1 = S1.CVRods{leg_index(i)}(2).Xs;
        z1 = g(7:4:4*(S1.CVRods{leg_index(i)}(2).nip),4);
        x1 = g(5:4:4*(S1.CVRods{leg_index(i)}(2).nip),4);
        y1 = g(6:4:4*(S1.CVRods{leg_index(i)}(2).nip),4);
        if i ==1
            index = find(z1<=constraint_height,1);
        else
            index = find(z1>=constraint_height,1);
        end
        alpha = (constraint_height - z1(index-1))/(z1(index)- z1(index - 1));
        roots(i) = Xs1(index-1)*(1-alpha) + Xs1(index)*alpha;
        xh = x1(index-1)*(1-alpha) + x1(index)*alpha;
        yh = y1(index-1)*(1-alpha) + y1(index)*alpha;
        hole_position(i,:) = [xh; yh; constraint_height];        
    end
end