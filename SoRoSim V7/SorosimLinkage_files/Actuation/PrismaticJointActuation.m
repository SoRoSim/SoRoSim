%Function that allows the user to specify prismatic joint control and actuation
%specifications (29.11.2024)

function [n_Pact,i_Pact,i_Pactq,WrenchControlledP,BP] = PrismaticJointActuation(Linkage,Update)
if nargin==1
    Update=false;
end
ndof              = Linkage.ndof;
B1                = zeros(ndof,1);

n_Pact            = 0;
i_Pact            = [];
i_Pactq           = [];
BP                = [];
WrenchControlledP = [];

dofi              = 1;

if ~Update
    for i = 1:Linkage.N %for each link

        VRods_i = Linkage.CVRods{i};

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype == 'P'

            close all
            Linkage.plotq0(i);

            quest  = ['Is the prismatic joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Prismatic Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Pact   = n_Pact+1;
                    i_Pact   = [i_Pact i];
                    i_Pactq  = [i_Pactq;dofi];
                    B1(dofi) = 1;
                    BP       = [BP,B1];
                    B1       = zeros(ndof,1);

                    quest    = 'Is the prismatic joint controlled by force or displacement?';
                    answer2  = questdlg(quest,'Control',...
                        'Force','Displacement','Force');

                    switch answer2
                        case 'Force'
                            WrenchControlledP = [WrenchControlledP;true];
                        case 'Displacement'
                            WrenchControlledP = [WrenchControlledP;false];
                    end
            end
        end
        dofi = dofi+VRods_i(1).dof;
        for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
else
    for i=1:Linkage.N %for each link

        VRods_i = Linkage.CVRods{i};

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='P'&&any(Linkage.i_jact==i)
            
            i_Pactq  = [i_Pactq dofi];
            B1(dofi) = 1;
            BP       = [BP,B1];
            B1       = zeros(ndof,1);

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
