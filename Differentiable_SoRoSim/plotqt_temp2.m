%Function for the plot of dynamic simulation
%Last modified by Anup Teejo Mathew - 25/05/2021
function plotqt_temp2(Linkage, t, qqd, options)

arguments
    Linkage
    t
    qqd
    % Options
    options.video_name = "Dynamics"
    options.video_path = "."
    options.record = true
    % options.video_ext = ".mp4"
end

close all

% Video Settings
video_ext = ".mp4";
cmap = turbo(Linkage.nb);   % or turbo/parula if you prefer

% Fullfile avoids conflicts with Ubuntu, since Windows use "\" and Ubuntu
% use "/".
video_file = fullfile(options.video_path, options.video_name + video_ext);

PlotParameters = Linkage.PlotParameters;

N         = Linkage.N;
g_ini     = Linkage.g_ini;
iLpre     = Linkage.iLpre;
nsig = Linkage.nsig;


tic
tmax        = max(t);

% With fullfile, we avoid conflicts with Ubuntu/Windows
FrameRate   = PlotParameters.FrameRateValue*2;
if options.record
    vid = VideoWriter(video_file, 'MPEG-4');
    vid.FrameRate = FrameRate;
    open(vid);
end

%Plot options

fh=figure(1);
ax = axes;
fh.Units='normalized';
% FigScale = PlotParameters.VideoResolution;
% FigScale(FigScale<0.1)=0.5;
% FigScale(FigScale>1)=1;
% FigLocation = (1-FigScale)/2;
% fh.OuterPosition=[FigLocation FigLocation FigScale FigScale];

set(gca,'CameraPosition',PlotParameters.CameraPosition,...
    'CameraTarget',PlotParameters.CameraTarget,...
    'CameraUpVector',PlotParameters.CameraUpVector,...
    'FontSize',18)

if PlotParameters.Light
    camlight(PlotParameters.Az_light,PlotParameters.El_light)
end

axis equal
grid on
hold on
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

% Set all text elements to use LaTeX interpreter
set(get(gca, 'Title'), 'Interpreter', 'latex');
set(get(gca, 'XLabel'), 'Interpreter', 'latex');
set(get(gca, 'YLabel'), 'Interpreter', 'latex');
set(get(gca, 'ZLabel'), 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');

set(gca,'FontSize',12)

set(gcf, 'Renderer', 'OpenGL');


for ib = 1:Linkage.nb
    Linkage.RobotBodies(ib).attachHandles(ax);
end
% Linkage.EnvironmentBodies(1).attachHandles(ax);
% 
% Linkage.EnvironmentBodies(1).setPose(eye(4));
% Linkage.EnvironmentBodies(1).hGeom.FaceAlpha = 1;
% Linkage.EnvironmentBodies(1).hGeom.FaceColor = [0.7 0.7 0.7];

hText = findobj(gca, 'Type', 'Text');
set(hText, 'Interpreter', 'latex');

axis tight
grid off

lighting phong; camlight headlight; material shiny
set(gcf,'Renderer','opengl')

axis ([PlotParameters.XLim PlotParameters.YLim PlotParameters.ZLim]/2);


set(gcf,'Renderer','opengl');
set(gcf,'GraphicsSmoothing','on');   % optional
set(ax,'SortMethod','depth');   % important with transparency

axis off;
set(gca,'Visible','off');
set(gcf,'Color',[1 1 1]);
set(gca,'Color',[1 1 1]);



%view(0,0)
drawnow


for tt=0:1/FrameRate:tmax

    g   = zeros(4*nsig,4);
    i_sig = 1;

    delete(findobj(ax, 'Type', 'quiver', 'Tag', 'contact_force'));
    delete(findobj(ax, 'Type', 'quiver', 'Tag', 'towards_origin'));

    %delete(findobj('type', 'patch'));
    title(strcat('t= ',num2str(tt)))
   
    qqdtt = interp1(t,qqd,tt);
    q     = qqdtt(1:Linkage.ndof)';
    
    dof_start = 1;
    g_tip    = repmat(eye(4),N,1);
    
    for i=1:N % number of links
        
        if iLpre(i)>0
            g_here=g_tip((iLpre(i)-1)*4+1:iLpre(i)*4,:)*g_ini((i-1)*4+1:i*4,:);
        else
            g_here=g_ini((i-1)*4+1:i*4,:);
        end

        g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
        i_sig = i_sig+1;
        
        %joint
        dof_here   = Linkage.CVRods{i}(1).dof;
        q_here     = q(dof_start:dof_start+dof_here-1);
        Phi_here   = Linkage.CVRods{i}(1).Phi;
        xi_star    = Linkage.CVRods{i}(1).xi_star;

        if dof_here==0 %fixed joint (N)
            g_joint    = eye(4);
        else
            xi         = Phi_here*q_here+xi_star;
            g_joint    = variable_expmap_g(xi);
        end
        g_here     = g_here*g_joint;

        
        
        if tt < 0.1
            % body origin in world
            p = g_here(1:3,4);      % world coordinates
            v = -p;             % vector pointing toward origin
        
            % optional: scale for visualization
            L = norm(v);
            if L > 0
                v = v / L *3;   % cap arrow length at 0.5 (tune)
            end
        
            quiver3( ...
                p(1), p(2), p(3), ...
                v(1), v(2), v(3), ...
                0, ...
                'Color','r', ...
                'LineWidth',1.5, ...
                'MaxHeadSize',1.0, ...
                'Tag','towards_origin');
        end

        
        n_r   = Linkage.VLinks(Linkage.LinkIndex(i)).n_r;
        if Linkage.VLinks(Linkage.LinkIndex(i)).CS=='R'
            n_r=5;
        end
        n_l   = Linkage.VLinks(Linkage.LinkIndex(i)).n_l;
        color = Linkage.VLinks(Linkage.LinkIndex(i)).color;
        alpha = Linkage.VLinks(Linkage.LinkIndex(i)).alpha;
        
        if Linkage.VLinks(Linkage.LinkIndex(i)).L>0
        if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
            L          = Linkage.VLinks(Linkage.LinkIndex(i)).L;
            gi         = Linkage.VLinks(Linkage.LinkIndex(i)).gi;
            g_here     = g_here*gi;

            g((i_sig-1)*4+1:i_sig*4,:)  = g_here;
            i_sig = i_sig+1;
            
            if ~Linkage.VLinks(Linkage.LinkIndex(i)).CPF
                Xr         = linspace(0,L,n_l);
                g_hereR    = g_here*[eye(3) [-Linkage.VLinks(Linkage.LinkIndex(i)).cx;0;0];0 0 0 1]; 
                dx         = Xr(2)-Xr(1);

                Xpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
                Ypatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
                Zpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
                i_patch = 1;

                [y,z] = computeBoundaryYZ(Linkage.VLinks(Linkage.LinkIndex(i)),0);
                pos  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r

                pos_here = g_hereR*pos;
                x_here   = pos_here(1,:);
                y_here   = pos_here(2,:);
                z_here   = pos_here(3,:);

                Xpatch(:,i_patch) = x_here';
                Ypatch(:,i_patch) = y_here';
                Zpatch(:,i_patch) = z_here';
                i_patch           = i_patch+1;

                x_pre    = x_here;
                y_pre    = y_here;
                z_pre    = z_here;

                for ii=2:n_l

                    [y,z] = computeBoundaryYZ(Linkage.VLinks(Linkage.LinkIndex(i)),Xr(ii)/L);
                    pos  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r

                    g_hereR  = g_hereR*[eye(3) [dx;0;0];0 0 0 1];
                    pos_here = g_hereR*pos;
                    x_here   = pos_here(1,:);
                    y_here   = pos_here(2,:);
                    z_here   = pos_here(3,:);

                    %Plotting rigid link
                    for jj=1:n_r-1

                        Xpatch(1:5,i_patch)   = [x_pre(jj) x_here(jj) x_here(jj+1) x_pre(jj+1) x_pre(jj)]';
                        Xpatch(6:end,i_patch) = x_pre(jj)*ones(n_r-5,1);
                        Ypatch(1:5,i_patch)   = [y_pre(jj) y_here(jj) y_here(jj+1) y_pre(jj+1) y_pre(jj)]';
                        Ypatch(6:end,i_patch) = y_pre(jj)*ones(n_r-5,1);
                        Zpatch(1:5,i_patch)   = [z_pre(jj) z_here(jj) z_here(jj+1) z_pre(jj+1) z_pre(jj)]';
                        Zpatch(6:end,i_patch) = z_pre(jj)*ones(n_r-5,1);
                        i_patch = i_patch+1;

                    end

                    x_pre    = x_here;
                    y_pre    = y_here;
                    z_pre    = z_here;

                end

                Xpatch(:,i_patch) = x_here';
                Ypatch(:,i_patch) = y_here';
                Zpatch(:,i_patch) = z_here';

                %patch(Xpatch,Ypatch,Zpatch,color,'EdgeColor','none','FaceAlpha',alpha);
                % deterministic colors per body
                
                
                Linkage.RobotBodies(i).setPose(g_here);
                Linkage.RobotBodies(i).hGeom.FaceAlpha = 1;
                Linkage.RobotBodies(i).hGeom.FaceColor = cmap(i, :);

            else
                CustomShapePlot(g_here);
            end
            gf     = Linkage.VLinks(Linkage.LinkIndex(i)).gf;
            g_here = g_here*gf;
            
        end
        end
        
        if ~Linkage.OneBasis
            dof_start = dof_start+dof_here;
        end
        
            %=============================================================================
        for j=1:(Linkage.VLinks(Linkage.LinkIndex(i)).npie)-1 % for each piece
            
            dof_here   = Linkage.CVRods{i}(j+1).dof;
            Type       = Linkage.CVRods{i}(j+1).Type;
            q_here     = q(dof_start:dof_start+dof_here-1);
            xi_starfn  = Linkage.CVRods{i}(j+1).xi_starfn;
            gi         = Linkage.VLinks(Linkage.LinkIndex(i)).gi{j};
            Phi_dof    = Linkage.CVRods{i}(j+1).Phi_dof;
            Phi_odr    = Linkage.CVRods{i}(j+1).Phi_odr;
            Phi_h      = Linkage.CVRods{i}(j+1).Phi_h;
            ld         = Linkage.VLinks(Linkage.LinkIndex(i)).ld{j};
            g_here     = g_here*gi;
               
            Xs          = linspace(0,1,n_l);
            H           = Xs(2)-Xs(1);
            Z           = (1/2)*H;          % Zanna quadrature coefficient

            Xpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
            Ypatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
            Zpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
            i_patch = 1;
            
            %cross sectional shape Circular, Rectangular, and Ellipsoidal
            [y,z] = computeBoundaryYZ(Linkage.VLinks(Linkage.LinkIndex(i)),0,j);
            pos  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r

            pos_here = g_here*pos;
            x_here   = pos_here(1,:);
            y_here   = pos_here(2,:);
            z_here   = pos_here(3,:);

            Xpatch(:,i_patch) = x_here';
            Ypatch(:,i_patch) = y_here';
            Zpatch(:,i_patch) = z_here';
            i_patch           = i_patch+1;

            x_pre = x_here;
            y_pre = y_here;
            z_pre = z_here;
            
            for ii=1:n_l-1
                
                %cross sectional shape Circular, Rectangular, and Ellipsoidal
                [y,z] = computeBoundaryYZ(Linkage.VLinks(Linkage.LinkIndex(i)),Xs(ii+1),j);
                pos  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r
                
                X   = Xs(ii);
                X_Z = X+Z;
                
                xi_Zhere  = xi_starfn(X_Z);
                Phi_Scale = diag([1/ld 1/ld 1/ld 1 1 1]);
                
                if ~isempty(q_here)
                    if strcmp(Type,'FEM Like')
                        SubClass  = Linkage.CVRods{i}(j+1).SubClass;
                        xi_Zhere  = xi_Zhere+Phi_Scale*Phi_h(X_Z,Phi_dof,Phi_odr,SubClass)*q_here;
                    elseif strcmp(Type,'Custom Independent')
                        xi_Zhere  = xi_Zhere+Phi_Scale*Phi_h(X_Z)*q_here;
                    else
                        xi_Zhere  = xi_Zhere+Phi_Scale*Phi_h(X_Z,Phi_dof,Phi_odr)*q_here;
                    end
                end
                
                Gamma_here = H*ld*xi_Zhere;

                gh         = variable_expmap_g(Gamma_here);
                g_here     = g_here*gh;
                
                pos_here = g_here*pos;
                x_here   = pos_here(1,:);
                y_here   = pos_here(2,:);
                z_here   = pos_here(3,:);


                for jj=1:n_r-1

                    Xpatch(1:5,i_patch)   = [x_pre(jj) x_here(jj) x_here(jj+1) x_pre(jj+1) x_pre(jj)]';
                    Xpatch(6:end,i_patch) = x_pre(jj)*ones(n_r-5,1);
                    Ypatch(1:5,i_patch)   = [y_pre(jj) y_here(jj) y_here(jj+1) y_pre(jj+1) y_pre(jj)]';
                    Ypatch(6:end,i_patch) = y_pre(jj)*ones(n_r-5,1);
                    Zpatch(1:5,i_patch)   = [z_pre(jj) z_here(jj) z_here(jj+1) z_pre(jj+1) z_pre(jj)]';
                    Zpatch(6:end,i_patch) = z_pre(jj)*ones(n_r-5,1);
                    i_patch = i_patch+1;

                end
                
                x_pre = x_here;
                y_pre = y_here;
                z_pre = z_here;
                
            end

            Xpatch(:,i_patch) = x_here';
            Ypatch(:,i_patch) = y_here';
            Zpatch(:,i_patch) = z_here';

            patch(Xpatch,Ypatch,Zpatch,color,'EdgeColor','none','FaceAlpha',alpha);
            
            gf     = Linkage.VLinks(Linkage.LinkIndex(i)).gf{j};
            g_here = g_here*gf;
            
            if ~Linkage.OneBasis
                dof_start = dof_start+dof_here;
            end
            
        end
        g_tip((i-1)*4+1:i*4,:) = g_here;

    end

    for icp = 1:Linkage.ncp

        i_sig1 = Linkage.Pairs(icp).body1.i_sig;
        i_sig2 = Linkage.Pairs(icp).body2.i_sig;
    
        g1 = g((i_sig1-1)*4+1:i_sig1*4,:);
        g2 = g((i_sig2-1)*4+1:i_sig2*4,:);
    
        out = Linkage.Pairs(icp).solveNarrowPhase(g1, g2);
    
        if out.alpha < 1+2*1e-3
            % relative transform (frame2 expressed in frame1)
            g_12 = Linkage.Pairs(icp).get_relative(g1, g2);
            r12  = g_12(1:3,4);                    % frame 1
    
            % penetration proxy (positive)
            delta = (1/out.alpha - 1) * norm(r12);
            delta = abs(delta);
            fn = Linkage.penalty.k_n * delta^Linkage.penalty.e_n;
    
            % normal from body1 implicit gradient (frame 1)
            grad = Linkage.Pairs(icp).body1.get_grad(out.x);
            n = grad / norm(grad);
    
            % force on body1 in frame 1
            fc = - fn * n; %(repulsion)

            % ---- plotting contact forces ----
            
            R = g1(1:3,1:3);
            p = g1(1:3,4);
            
            % contact point in world frame
            xw = R*out.x + p;
            
            % force vector in world frame
            fcw = R*fc*3*1e-4;

            c1 = cmap(Linkage.Pairs(icp).body1.id, :);   % 1x3 RGB
            c2 = cmap(Linkage.Pairs(icp).body2.id, :);   % 1x3 RGB

            quiver3( ...
                    xw(1), xw(2), xw(3), ...
                    fcw(1), fcw(2), fcw(3), ...
                    0, ...
                    'Color', c1, ...
                    'MaxHeadSize', 1.0, ...
                    'LineWidth', 1.5, ...
                    'Tag', 'contact_force' );

            quiver3( ...
                    xw(1), xw(2), xw(3), ...
                   -fcw(1), -fcw(2), -fcw(3), ...
                    0, ...
                    'Color', c2, ...
                    'LineStyle','--', ...
                    'MaxHeadSize', 1.0, ...
                    'LineWidth', 1.5, ...
                    'Tag', 'contact_force' );


        end
    end
    
    drawnow limitrate nocallbacks;
    if options.record
        frame = getframe(gcf);
        frame.cdata = imresize(frame.cdata, [ceil(size(frame.cdata,1)/2)*2, ceil(size(frame.cdata,2)/2)*2]);
        writeVideo(vid,frame);
    end
end

if options.record
    close(vid);

    % Play the video only if it was recorded
    answer = questdlg('Play output video in MATLAB?','Grapical Output', ...
	    'Yes','No','Yes');
    
    if strcmp('Yes',answer)
        implay(video_file)
    end
end