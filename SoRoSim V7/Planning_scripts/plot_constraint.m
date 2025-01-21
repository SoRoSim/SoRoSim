function plot_constraint(constraint_surface)
    theta = linspace(0,2*pi);
    xh1 = constraint_surface.radius*cos(theta) +constraint_surface.hole_1(1);
    yh1 = constraint_surface.radius*sin(theta) + constraint_surface.hole_1(2);
    zh1 = ones(1,length(theta))*constraint_surface.height;
    
    xh2 = constraint_surface.radius*cos(theta) + constraint_surface.hole_2(1);
    yh2 = constraint_surface.radius*sin(theta) + constraint_surface.hole_2(2);
    zh2 = ones(1,length(theta))*constraint_surface.height;
    
    plot3(xh1, yh1, zh1,'r')
    hold on
    plot3(xh2, yh2, zh2,'r');
end