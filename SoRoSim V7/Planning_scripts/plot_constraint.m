function plot_constraint(hole_pos, radius, leg_index)
    theta = linspace(0,2*pi);
    for i =1:length(leg_index)
        xh1 = radius*cos(theta) +hole_pos(i,1);
        yh1 = radius*sin(theta) + hole_pos(i,2);
        zh1 = ones(1,length(theta))*hole_pos(i,3);
        plot3(xh1, yh1, zh1,'r')
        hold on
    end
    hold off
end