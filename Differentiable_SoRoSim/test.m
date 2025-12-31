clear; close all; clc;

% ---------- Body 1 (polytope) ----------
shape_id = 2;

A1 = 1/sqrt(3)*[ 1   2    1;   
                 1  -1   -1;   
                -1   1   -1;    
                -1  -1    1;
                -1  -4   -1;   
                -2   1    1;   
                 1  -3    1;    
                 1   1    4];

b1 = [1; 1; 1; 1; 5/3; 5/3; 5/3; 5/3];

beta = 20;

params1 = idcol_make_params(shape_id, beta, A1, b1);
B1 = SorosimContactBody(1, shape_id, params1);

% ---------- Body 2 (another polytope, you can change A2/b2) ----------
A2 = 1/sqrt(3)*[ 1   1    1;   
                 1  -1   -1;   
                -1   1   -1;    
                -1  -1    1;
                -1  -1   -1;   
                -1   1    1;   
                 1  -1    1;    
                 1   1   -1];

b2 = [1; 1; 1; 1; 5/3; 5/3; 5/3; 5/3];

params2 = idcol_make_params(shape_id, beta, A2, b2);
B2 = SorosimContactBody(2, shape_id, params2);

% ---------- Plot ----------
figure('Color','w');
ax = axes; hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); view(3);
xlabel('x'); ylabel('y'); zlabel('z');

B1.attachHandles(ax);
B2.attachHandles(ax);

camlight headlight; lighting gouraud;
set(B1.hGeom,'EdgeColor','none','FaceAlpha',0.35);
set(B2.hGeom,'EdgeColor','none','FaceAlpha',0.35);

% Give different colors (optional)
set(B1.hGeom,'FaceColor',[1.0 0.6 0.2]);
set(B2.hGeom,'FaceColor',[0.2 0.6 1.0]);

% ---------- Place bodies ----------
g1 = eye(4);
g2 = eye(4);
g2(1:3,4) = [2; 0; 0];   % start far away

% If you have setPose:
if ismethod(B1,'setPose')
    B1.setPose(g1);
    B2.setPose(g2);
else
    set(B1.hT,'Matrix',g1);
    set(B2.hT,'Matrix',g2);
end

% ---------- Create pair ----------
P12 = SorosimContactPair(B1, B2, 1);

% ---------- Animate B2 and test broadphase ----------
N = 120;
for k = 1:N
    % Move B2 along x toward B1 and back
    x = 5 - 10.0*(k-1)/(N-1);  
    g2(1:3,4) = [x; 0; 0];

    if ismethod(B2,'setPose')
        B2.setPose(g2);
    else
        set(B2.hT,'Matrix',g2);
    end

    tf = P12.broadphase(g1, g2);

    % quick visual feedback in title
    if tf
        title(ax, 'broadphase\_active = true');
    else
        title(ax, 'broadphase\_active = false');
    end

    % drawnow limitrate
    drawnow;        % FORCE repaint (not limitrate)
    pause(0.01);    % tiny delay so you can see it
end
