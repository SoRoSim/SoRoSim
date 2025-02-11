%Function that allows the user to specify cylinderical joint control and actuation
%specifications (29.11.2024)

function [n_Cact,i_Cact,i_Cactq,WrenchControlledC] = CylindricalJointActuation(Linkage,Update)
if nargin==1
    Update=false;
end
n_Cact            = 0;
i_Cact            = [];
i_Cactq           = [];
WrenchControlledC = [];

dofi              = 1;

if ~Update
    for i=1:Linkage.N

        VRods_i = Linkage.CVRods{i};

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype == 'C'

            close all
            Linkage.plotq0(i);

            quest  = ['Is the cylindircal joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Cylindrical Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Cact  = n_Cact+2;
                    i_Cact  = [i_Cact i];
                    i_Cactq = [i_Cactq dofi dofi+1];

                    quest = 'Is the cylindrical joint controlled by wrenches or joint coordinates (qs)?';
                    answer2 = questdlg(quest,'Control',...
                        'Wrench','q','Wrench');

                    switch answer2
                        case 'Wrench'
                            WrenchControlledC = [WrenchControlledC true true];
                        case 'q'
                            WrenchControlledC = [WrenchControlledC false false];
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

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='C'&&any(Linkage.i_jact==i)
            
            i_Cactq  = [i_Cactq dofi dofi+1];

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
