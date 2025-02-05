%Function that calculates the screw velocity (eta) at every significant point
%in SI units (last modified 05/02/2025, Anup)
function xi = ScrewStrain(Linkage,q,i_here,j_here) %i_here is link index, j_here is division (0 for joint)

if isrow(q)
    q=q';
end

N = Linkage.N;

full = false;
if nargin==4
    if j_here==0
        nsig = 1;
    else
        nsig = Linkage.CVTwists{i_here}(j_here+1).nip; %j_here>1 is allowed only for soft links
    end
elseif nargin==3
    if Linkage.VLinks(Linkage.LinkIndex(i_here)).linktype=='s'
        nsig = 1;
        for j=1:Linkage.VLinks(Linkage.LinkIndex(i_here)).npie-1
            nsig = nsig+Linkage.CVTwists{i_here}(j+1).nip;
        end
    else
        nsig = 2; %joint and CM
    end
else
    full  = true;
    nsig  = Linkage.nsig;
end
xi        = zeros(6*nsig,1);
dof_start = 1; %starting dof of current piece
i_sig     = 1;

for i = 1:N
    
    %Joint
    dof_here = Linkage.CVTwists{i}(1).dof;
    q_here   = q(dof_start:dof_start+dof_here-1);
    Phi_here = Linkage.CVTwists{i}(1).Phi;
    xi_star  = Linkage.CVTwists{i}(1).xi_star;
    
    if dof_here == 0 %fixed joint (N)
        xi_here = zeros(6,1);
    else
        xi_here  = Phi_here*q_here+xi_star;
    end
    if full||(i==i_here&&nargin==3)||(i==i_here&&j_here==0)
        xi((i_sig-1)*6+1:i_sig*6) = xi_here;
        i_sig                     = i_sig+1;
    end
    
    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype == 'r'
        
        if full||(i==i_here&&nargin==3)
            xi((i_sig-1)*6+1:i_sig*6) = zeros(6,1); % doesnt have a meaning
            i_sig                     = i_sig+1;
        end

    end
    
    dof_start = dof_start+dof_here;
    
    for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
        
        dof_here = Linkage.CVTwists{i}(j+1).dof;
        q_here   = q(dof_start:dof_start+dof_here-1);
        xi_star  = Linkage.CVTwists{i}(j+1).xi_star;
        nip      = Linkage.CVTwists{i}(j+1).nip;
        Phi      = Linkage.CVTwists{i}(j+1).Phi;

        
        for ii = 1:nip
            
            xi_here = xi_star(6*(ii-1)+1:6*ii,1);

            Phi_here  = Phi(6*(ii-1)+1:6*ii,:);%note this step
            if dof_here>0
                xi_here = Phi_here*q_here+xi_here; 
            end
            
            if full||(i==i_here&&nargin==3)||(i==i_here&&j==j_here)
                xi((i_sig-1)*6+1:i_sig*6) = xi_here;
                i_sig                     = i_sig+1;
            end
            
        end
        
        dof_start  = dof_start+dof_here;
    end
    
end
end

