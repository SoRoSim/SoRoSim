%If dFact_dq is unavailable keep it [] and turn off Jacobian during simulation
%Last modified by Anup Teejo Mathew 05/02/2025
function [Fact,dFact_dq] = CustomActuation(Linkage,q,g,J,t,qd,Jdot)

%q and qd: joint coordinates and their time derivatives, 
%g, J, Jd, and eta: transformation matrix, Jacobian, time derivative of jacobian, and screw velocity at every significant point of the linkage
%t:  time

%Fact should be 6*n column vector where n is the total number of gaussian
%points of all soft divisions including the start and end point of a division (nip).
%(Example: linkage with 2 soft links and 1 rigid link (n=nip1+nip2)
%Fact should be arranged according to the order of precedence of Link
%numbers

% Significant points: 1 for every joint, 1 at the center of the rigid link, 1
% at the start and end of every soft link and 1 for each Gaussian points

% J   = S.Jacobian(q);         %geometric jacobian of the linkage calculated at every significant points
% g   = S.FwdKinematics(q);    %transformation matrix of the linkage calculated at every significant points
% eta = S.ScrewVelocity(q,qd); %Screwvelocity of the linkage calculated at every significant points
% J   = S.Jacobiandot(q,qd);   %time derivative of geometric jacobian of the linkage calculated at every significant points


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CUSTOM INTERNAL ACTUATION, Tentacle actuation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n=0;
for i=1:Linkage.N
    for j=1:Linkage.VLinks(i).npie-1
        n=n+Linkage.CVRods{i}(j+1).nip;
    end
end
Fact = zeros(6*n,1);
dFact_dq = zeros(6*n,Linkage.ndof);

%%

tf = 2;

Xs = Linkage.CVTwists{1}(2).Xs;
% nip = Linkage.CVRods{1}(2).nip;
% Vect_M =zeros(1,nip);

k = 1.36;
L0 = 0.20;

epsilon = 53.4-10*t;


E = 1e4;

rbase = 0.01;
rtip = 0.001;
Xe = 0.5+0.4*t/tf;

for ii = 1:length(Xs) 
    
    r_here = rbase - Xs(ii)*(rbase - rtip);
    Jy = (pi/4)*(r_here^4);
    M = E*Jy*k*epsilon./(1+(epsilon*L0*(Xs(ii)-Xe))^2);
    % strain = k*epsilon./(1+(epsilon*L0*(Xs(ii)-Xe))^2);
    % Vect_M = M(ii);
    Fact(ii*6-5:ii*6) = [0 -M 0 0 0 0]';
end

end
