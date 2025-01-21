%% Make video of trajectory following

trajectory = 'z_spiral_with_rotation';
load(strcat(trajectory,'.mat'));
%%
trajectory = "obstacle avoidance"
mov = VideoWriter(strcat(trajectory, '.mp4'),'MPEG-4');
mov.FrameRate = 15;
open(mov)
%%framing


for i =1:length(gd)/4
    q = qs(i,:);
    S1.plotq(q)
    g = S1.FwdKinematics(q);
    g_platform(4*i-3:4*i,1:4) = g(4*(length(S1.CVTwists{1}(2).Xs)+2)+1:4*(length(S1.CVTwists{1}(2).Xs)+3),:);
    g_base(4*i-3:4*i, 1:4) = g(1:4,1:4);
    g_tip(4*i-3:4*i,1:4) = g(69:72,1:4);%% generalize later
    
    xlim([min(gd(1:4:end,4))-0.5, max(gd(1:4:end, 4))+0.5])
    ylim([min(gd(2:4:end,4))-0.5, max(gd(2:4:end,4))+0.5])
    zlim([min(gd(3:4:end,4))-0.5, max(gd(3:4:end))+0.5])
    set(gca,'CameraPosition',[4.9972  -11.9867    6.7754]);
    set(gca,"CameraTarget", [0.5406    0.3500    0.8190]);
    plot3(g_obstacle(1,4), g_obstacle(2,4), g_obstacle(3,4),'*')
    plot3(gd(1:4:end,4),gd(2:4:end,4), gd(3:4:end,4),'--', 'Color','red')
    plotTransforms(se3(g_platform(4*i-3:4*i,1:4)), 'FrameSize',0.05)
    plotTransforms(se3(g_desired), 'FrameSize',0.05);
    plotTransforms(se3(g_tip(4*i-3:4*i,1:4)), 'FrameSize',0.05);
    plotTransforms(se3(g_base(4*i-3:4*i,1:4)),'FrameSize',0.05);
    F   = getframe(gcf);
    clf
    writeVideo(mov,F);

end
close(mov)