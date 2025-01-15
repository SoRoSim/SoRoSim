%Function that allows the user to specify helical joint control and actuation
%specifications (24.05.2021)

function [n_Hact,i_Hact,i_Hactq,WrenchControlledH,BqH] = HelicalJointActuation(S,Update)
if nargin==1
    Update=false;
end
ndof              = S.ndof;
B1                = zeros(ndof,1);

n_Hact            = 0;
i_Hact            = [];
i_Hactq           = [];
BqH               = [];
WrenchControlledH = [];

dofi              = 1;

if ~Update
    for i = 1:S.N %for each link

        VRods_i = S.CVRods{i};

        if S.VLinks(S.LinkIndex(i)).jointtype == 'H'

            close all
            S.plotq0(i);

            quest  = ['Is the Helical joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Helical Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Hact   = n_Hact+1;
                    i_Hact   = [i_Hact i];
                    i_Hactq  = [i_Hactq;dofi];
                    B1(dofi) = 1;
                    BqH      = [BqH,B1];
                    B1       = zeros(ndof,1);

                    quest    = 'Is the helical joint controlled by wrench or joint coordinate (q)?';
                    answer2  = questdlg(quest,'Control',...
                        'Wrench','q','Wrench');

                    switch answer2
                        case 'Wrench'
                            WrenchControlledH = [WrenchControlledH;true];
                        case 'q'
                            WrenchControlledH = [WrenchControlledH;false];
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

        if S.VLinks(S.LinkIndex(i)).jointtype=='H'&&any(S.i_jact==i)
            
            i_Hactq  = [i_Hactq dofi];
            B1(dofi) = 1;
            BqH      = [BqH,B1];
            B1       = zeros(ndof,1);

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:S.VLinks(S.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
