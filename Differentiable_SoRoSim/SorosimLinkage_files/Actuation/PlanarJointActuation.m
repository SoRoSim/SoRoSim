%Function that allows the user to specify planar joint control and actuation
%specifications (24.05.2021)

function [n_Aact,i_Aact,i_Aactq,WrenchControlledA] = PlanarJointActuation(Linkage,Update)
if nargin==1
    Update=false;
end

n_Aact            = 0;
i_Aact            = [];
i_Aactq           = [];
WrenchControlledA = [];

dofi              = 1;

if ~Update
    for i=1:Linkage.N %for each link

        VRods_i = Linkage.CVRods{i};

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype == 'A'

            close all
            Linkage.plotq0(i);

            quest  = ['Is the planar joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Planar Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Aact  = n_Aact+3;
                    i_Aact  = [i_Aact i];
                    i_Aactq = [i_Aactq dofi dofi+1 dofi+2];

                    quest   = 'Is the planar joint controlled by wrenches or joint coordinates (qs)?';
                    answer2 = questdlg(quest,'Control',...
                        'Wrench','q','Wrench');

                    switch answer2
                        case 'Wrench'
                            WrenchControlledA = [WrenchControlledA true true true];
                        case 'q'
                            WrenchControlledA = [WrenchControlledA false false false];
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

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='A'&&any(Linkage.i_jact==i)
            
            i_Aactq  = [i_Aactq dofi dofi+1 dofi+2];

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
