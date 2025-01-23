%Function to calculate the custom external force (point force in local
%frame, if distributed multiply with quadrature weight)

%Last modified by Anup Teejo Mathew 11/12/2024

function [Fext,dFext_dq]=CustomExtForce(Linkage,q,g,J,t,qd,Jdot)

%%%%NOTE%%%%
%Tr: Linkage element,
%q and qd: joint coordinates and their time derivatives,
%g, J, Jd, and eta: transformation matrix, Jacobian, time derivative of jacobian, and screw velocity at every significant point of the linkage
%t: time
%Currently applicable only for independent bases

%Fext should be 6*nsig column vector. If distributed wrench, multiply with
%corresponding quadrature weight to make it a point wrench

% Significant points: 1 for every joint (at X=1), 1 at the center of the rigid link, for soft links at every integration points

% J   = S.Jacobian(q);         %geometric jacobian of the linkage calculated at every significant points
% g   = S.FwdKinematics(q);    %transformation matrix of the linkage calculated at every significant points
% eta = S.ScrewVelocity(q,qd); %Screwvelocity of the linkage calculated at every significant points
% J   = S.Jacobiandot(q,qd);   %time derivative of geometric jacobian of the linkage calculated at every significant points

%%%END%%%

nsig    = Linkage.nsig; %everything except rigid joints
Fext = zeros(6*nsig,1);
dFext_dq = zeros(6*nsig,Linkage.ndof);

%make changes from here

%% Fluid Interaction

DL     = Linkage.CP1;
G      = Linkage.G;

i_sig    = 1;
i_sig_nj = 1;

Rho_water  = 1000;

for i=1:Linkage.N 
    i_sig    = i_sig+1;%joint
    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
        Rho      = Linkage.VLinks(Linkage.LinkIndex(i)).Rho;
        M_here   = Linkage.VLinks(Linkage.LinkIndex(i)).M;
        g_here   = g((i_sig-1)*4+1:i_sig*4,:);
        eta_here = eta((i_sig-1)*6+1:i_sig*6);
        DL_here  = DL((i_sig_nj-1)*6+1:i_sig_nj*6,:);
        
        M_here(1:3,1:3)=zeros(3,3);
        
        if Linkage.Gravity
            Fext((i_sig_nj-1)*6+1:i_sig_nj*6) = -Rho_water/Rho*M_here*dinamico_Adjoint(ginv(g_here))*G-DL_here*norm(eta_here(4:6))*eta_here;
        else
            Fext((i_sig_nj-1)*6+1:i_sig_nj*6) = -DL_here*norm(eta_here(4:6))*eta_here;
        end

        
        i_sig    = i_sig+1;
        i_sig_nj = i_sig_nj+1;
    end
    for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
        Rho = Linkage.VLinks(Linkage.LinkIndex(i)).Rho;
        Ms  = Linkage.CVTwists{i}(j+1).Ms;
        nip = Linkage.CVTwists{i}(j+1).nip;
        
        for ii=1:nip
    
            M_here   = Ms((ii-1)*6+1:ii*6,:);
            g_here   = g((i_sig-1)*4+1:i_sig*4,:);
            eta_here = eta((i_sig-1)*6+1:i_sig*6);
            DL_here  = DL((i_sig_nj-1)*6+1:i_sig_nj*6,:);
            
            if i==8||i==9||i==12||i==13
                DL_here(5,6)=-DL_here(5,6);
                DL_here(6,5)=-DL_here(6,5);
            end
            
            if Linkage.Gravity
                Fext((i_sig_nj-1)*6+1:i_sig_nj*6)  = -Rho_water/Rho*M_here*dinamico_Adjoint(ginv(g_here))*G-DL_here*norm(eta_here(4:6))*eta_here;%
            else
                Fext((i_sig_nj-1)*6+1:i_sig_nj*6)  = -DL_here*norm(eta_here(4:6))*eta_here;%
            end

            i_sig    = i_sig+1;
            i_sig_nj = i_sig_nj+1;
        end

    end
end

end