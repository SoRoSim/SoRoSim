%Function that allows the user to specify prismatic joint control and actuation
%specifications (29.11.2024)

function [n_Pact,i_Pact,i_Pactq,WrenchControlledP,BqP] = PrismaticJointActuation(S,Update)
if nargin==1
    Update=false;
end
ndof              = S.ndof;
B1                = zeros(ndof,1);

n_Pact            = 0;
i_Pact            = [];
i_Pactq           = [];
BqP               = [];
WrenchControlledP = [];

dofi              = 1;

if ~Update
    for i = 1:S.N %for each link

        VRods_i = S.CVRods{i};

        if S.VLinks(S.LinkIndex(i)).jointtype == 'P'

            close all
            S.plotq0(i);

            quest  = ['Is the prismatic joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Prismatic Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Pact   = n_Pact+1;
                    i_Pact   = [i_Pact i];
                    i_Pactq  = [i_Pactq;dofi];
                    B1(dofi) = 1;
                    BqP      = [BqP,B1];
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
        for j = 1:S.VLinks(S.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
else
    for i=1:S.N %for each link

        VRods_i = S.CVRods{i};

        if S.VLinks(S.LinkIndex(i)).jointtype=='P'&&any(S.i_jact==i)
            
            i_Pactq  = [i_Pactq dofi];
            B1(dofi) = 1;
            BqP      = [BqP,B1];
            B1       = zeros(ndof,1);

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:S.VLinks(S.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
