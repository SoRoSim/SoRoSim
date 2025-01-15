%Function to plot a SorosimLink
%Last modified by Anup Teejo Mathew 28.11.2024
function plotLink(Link)

close all
figure
hold on
camlight
daspect([1 1 1]); % Data aspect ratio (1:1:1 for uniform scaling)
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Link Preview')

% Set all text elements to use LaTeX interpreter
set(get(gca, 'Title'), 'Interpreter', 'latex');
set(get(gca, 'XLabel'), 'Interpreter', 'latex');
set(get(gca, 'YLabel'), 'Interpreter', 'latex');
set(get(gca, 'ZLabel'), 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');

set(gca,'FontSize',14)


g_here = eye(4);
    
n_r   = Link.n_r;
n_l   = Link.n_l;
color = Link.color;
alpha = Link.alpha;
scale = Link.Lscale;

o0=[0,0,0];
oX=0.9*[scale,0,0];oY=0.9*[0 scale 0];oZ=0.9*[0 0 scale];

o0_here=(g_here*[o0';1])';
oX_here=(g_here*[oX';1])';
oY_here=(g_here*[oY';1])';
oZ_here=(g_here*[oZ';1])';

arrow3(o0_here(1:3),oX_here(1:3),'r-1',scale*5);
arrow3(o0_here(1:3),oY_here(1:3),'g-1',scale*5);
arrow3(o0_here(1:3),oZ_here(1:3),'b-1',scale*5);

if Link.L>0
    if Link.linktype=='r'
        
        L       = Link.L;
        gi      = Link.gi;
        g_here  = g_here*gi; %at CM

        o0_here=(g_here*[o0';1])';
        oX_here=(g_here*[oX';1])';
        oY_here=(g_here*[oY';1])';
        oZ_here=(g_here*[oZ';1])';
        
        arrow3(o0_here(1:3),oX_here(1:3),'r-1',scale*5);
        arrow3(o0_here(1:3),oY_here(1:3),'g-1',scale*5);
        arrow3(o0_here(1:3),oZ_here(1:3),'b-1',scale*5);
        
        if ~Link.CPF %not custom shape (custom plot function)
            Xr      = linspace(0,L,n_l);
            g_hereR = g_here*[eye(3) [-Link.cx;0;0];0 0 0 1]; 
            dx      = Xr(2)-Xr(1);
            Xpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
            Ypatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
            Zpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
            i_patch = 1;

            [y,z] = computeBoundaryYZ(Link,0);
            posH  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r

            pos_here = g_hereR*posH;
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

                [y,z] = computeBoundaryYZ(Link,Xr(ii)/L);
                posH   = [zeros(1,n_r);y;z;ones(1,n_r)];

                g_hereR  = g_hereR*[eye(3) [dx;0;0];0 0 0 1];
                pos_here = g_hereR*posH;
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

                x_pre    = x_here;
                y_pre    = y_here;
                z_pre    = z_here;


            end
            Xpatch(:,i_patch) = x_here';
            Ypatch(:,i_patch) = y_here';
            Zpatch(:,i_patch) = z_here';

            patch(Xpatch,Ypatch,Zpatch,color,'EdgeColor','none','FaceAlpha',alpha);
            
            gf = Link.gf;
            g_here = g_here*gf;
            o0_here=(g_here*[o0';1])';
            oX_here=(g_here*[oX';1])';
            oY_here=(g_here*[oY';1])';
            oZ_here=(g_here*[oZ';1])';
            
            arrow3(o0_here(1:3),oX_here(1:3),'r-1',scale*5);
            arrow3(o0_here(1:3),oY_here(1:3),'g-1',scale*5);
            arrow3(o0_here(1:3),oZ_here(1:3),'b-1',scale*5);
        else
            CustomShapePlot(g_here);
        end
    end
end

%=============================================================================
%Soft link pieces
for j=1:Link.npie-1
    
    gi     = Link.gi{j};
    ld     = Link.ld{j};
    g_here = g_here*gi;
    o0_here=(g_here*[o0';1])';
    oX_here=(g_here*[oX';1])';
    oY_here=(g_here*[oY';1])';
    oZ_here=(g_here*[oZ';1])';
    
    arrow3(o0_here(1:3),oX_here(1:3),'r-1',scale*5);
    arrow3(o0_here(1:3),oY_here(1:3),'g-1',scale*5);
    arrow3(o0_here(1:3),oZ_here(1:3),'b-1',scale*5);

    Xs = linspace(0,1,n_l);
    dx  = (Xs(2)-Xs(1))*ld;
    
    Xpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
    Ypatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
    Zpatch  = zeros(n_r,(n_r-1)*(n_l-2)+2);
    i_patch = 1;
    
    [y,z] = computeBoundaryYZ(Link,0,j);
    posH  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r
    
    pos_here = g_here*posH;
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

        [y,z] = computeBoundaryYZ(Link,Xs(ii+1),j);
        posH  = [zeros(1,n_r);y;z;ones(1,n_r)]; %homogeneous positions in local frame 4xn_r
        
        gh         = [eye(3) [dx;0;0];0 0 0 1];
        g_here     = g_here*gh;

        pos_here = g_here*posH;
        x_here     = pos_here(1,:);
        y_here     = pos_here(2,:);
        z_here     = pos_here(3,:);
        
        %Plotting soft link pieces
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

    patch(Xpatch,Ypatch,Zpatch,color,'EdgeColor','none','FaceAlpha',alpha);

    gf     = Link.gf{j};
    g_here = g_here*gf;
    o0_here=(g_here*[o0';1])';
    oX_here=(g_here*[oX';1])';
    oY_here=(g_here*[oY';1])';
    oZ_here=(g_here*[oZ';1])';
    
    arrow3(o0_here(1:3),oX_here(1:3),'r-1',scale*5);
    arrow3(o0_here(1:3),oY_here(1:3),'g-1',scale*5);
    arrow3(o0_here(1:3),oZ_here(1:3),'b-1',scale*5);
end

axis tight
view(3)
end