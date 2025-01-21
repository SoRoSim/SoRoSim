%% Visualize trajectory
g_0 = [0.0000         0   -1.0000    0.3
         0    1.0000         0         0
    1.0000         0    0.0000    0.6
         0         0         0    1.0000];
trajectory = "rotation_x";
[t,gd] = generate_trajectory(trajectory,5,g_0);

mov = VideoWriter(strcat(trajectory, '.mp4'),'MPEG-4');
mov.FrameRate = 15;
open(mov)
figure
for i =1:length(t)

    plotTransforms(se3(gd(4*i-3:4*i,1:4)), 'FrameSize',0.05)

    xlim([min(gd(1:4:end,4))-0.5, max(gd(1:4:end, 4))+0.5])
    ylim([min(gd(2:4:end,4))-0.5, max(gd(2:4:end,4))+0.5])
    zlim([min(gd(3:4:end,4))-0.5, max(gd(3:4:end))+0.5])
    set(gca,'CameraPosition',[5.7008  -18.7311   11.3153]);
    set(gca,"CameraTarget", [0.3000         0    1.6000]);

    F   = getframe(gcf);
    clf
    writeVideo(mov,F);
end
close(mov)