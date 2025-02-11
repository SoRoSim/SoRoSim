%Use this function to evaluate the kinetic, gravitational potential and
%elastic potential energy of the beam. Solve dynamics before runing this
%code
% load('FreeFall.mat')
load('DynamicsSolution.mat')

ndof   = S1.ndof;
nip    = S1.CVRods{1}(2).nip;
Ws     = S1.CVRods{1}(2).Ws;
Xs     = S1.CVRods{1}(2).Xs;
tt     = 0:0.01:t(end);
nt     = length(tt);
T      = zeros(nt,1);
Vg     = zeros(nt,1);
Ve     = zeros(nt,1);
ld     = S1.VLinks(1).ld{1};

Ms       = S1.CVRods{1}(2).Ms;
Es       = S1.CVRods{1}(2).Es;
Phi      = S1.CVRods{1}(2).Phi;
xi_star  = S1.CVRods{1}(2).xi_star;

for i=1:nt

    qqdt  = interp1(t,qqd,tt(i));
    eta   = S1.ScrewVelocity(qqdt(1:ndof)',qqdt(ndof+1:2*ndof)');
    g     = S1.FwdKinematics(qqdt(1:ndof)');
    i_sig = 3;

    for ii=2:nip-1
        
        eta_here = eta((i_sig-1)*6+1:i_sig*6);
        g_here   = g((i_sig-1)*4+1:i_sig*4,:);
        M_here   = Ms((ii-1)*6+1:ii*6,:);
        E_here   = Es((ii-1)*6+1:ii*6,:);
        Phi_here = Phi((ii-1)*6+1:ii*6,:); 

        xi_starhere = xi_star(6*(ii-1)+1:6*(ii),1);
        xi_here     = Phi_here*qqdt(1:ndof)'+xi_starhere;

        T(i)  = T(i)+Ws(ii)*ld*0.5*eta_here'*M_here*eta_here;
        Vg(i) = Vg(i)+Ws(ii)*ld*M_here(6,6)*-S1.G(6)*g_here(3,4);
        Ve(i) = Ve(i)+Ws(ii)*ld*0.5*(xi_here-xi_starhere)'*E_here*(xi_here-xi_starhere);
        i_sig = i_sig+1;
    end

end
close all
plot(tt,T,'r','LineWidth',1.5)
hold on
plot(tt,Vg,'g','LineWidth',1.5)
plot(tt,Ve,'b','LineWidth',1.5)
E=T+Vg+Ve;
plot(tt,E,'k','LineWidth',1.5)
legend('Kinetic energy','Gravitational potential energy','Elastic potential energy','Total energy')
xlabel('Time in s')
ylabel('Energy in J')
set(gca,'FontSize',18)