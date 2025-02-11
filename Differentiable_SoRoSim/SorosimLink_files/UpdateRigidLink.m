%This function updates dependent properties when a property is changed
%(27.11.2024)

function UpdateRigidLink(Link) 

if Link.CS == 'R'

    [M,cx] = RigidBodyProperties(Link.CS,Link.L,Link.Rho,Link.h,Link.w);

elseif Link.CS=='C'

    [M,cx] = RigidBodyProperties(Link.CS,Link.L,Link.Rho,Link.r);

elseif Link.CS=='E'

    [M,cx] = RigidBodyProperties(Link.CS,Link.L,Link.Rho,Link.a,Link.b);

end

Link.M = M;
Link.cx = cx;
Link.gi = [eye(3),[cx;0;0];[0 0 0 1]];
Link.gf = [eye(3),[Link.L-cx;0;0];[0 0 0 1]];

Lscale = (M(4,4)/Link.Rho)^(1/3); %scale with cube root of volume
Lscale(Lscale==0)=0.1;
Link.Lscale = Lscale;

end

%eof