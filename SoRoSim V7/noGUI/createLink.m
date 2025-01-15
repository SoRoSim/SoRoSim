%This is for those who hate GUI

Link1 = SorosimLink('empty'); %create an empty class element

%To create a rigid link
Link1.jointtype='R';
Link1.linktype='r';
Link1.npie=1;

CS = 'C'; %'C' for circular, 'R' for circular, 'E' for circular
L = 0.3;
Rho = 1000;
r = @(X1)0.03; %function of normalzied X. X1 in [0 1]

[M,cx] = RigidBodyProperties(CS,L,Rho,r); %for circular cross section
% [M,cx] = RigidBodyProperties(CS,L,Rho,h,w); %for rectangular cross section
% [M,cx] = RigidBodyProperties(CS,L,Rho,a,b); %for elliptical cross section


Link1.L= L;
Link1.CS= 'C';
Link1.r= @(X1)0.03;
Link1.h= [];
Link1.w= [];
Link1.a= [];
Link1.b= [];
Link1.cx= cx;
Link1.gi = eye(4);
Link1.gf = eye(4);
Link1.M= M;

Link1.Rho= Rho;
Link1.Kj= [];
Link1.Dj= [];

Link1.n_l= 25;
Link1.n_r= 18; %should be 5 for rectanglular cross section
Link1.color= [0.9572 0.4854 0.8003];
Link1.alpha= 1;
Link1.CPF= false;
Link1.PlotFn= @(g)CustomShapePlot(g);
Link1.Lscale= 0.0947;