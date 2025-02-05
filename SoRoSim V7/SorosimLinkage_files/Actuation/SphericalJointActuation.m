%Function that allows the user to specify spherical joint control and actuation
%specifications (29.11.2024)

function [n_Sact,i_Sact,i_Sactq,WrenchControlledS] = SphericalJointActuation(Linkage,Update)
if nargin==1
    Update=false;
end
n_Sact            = 0;
i_Sact            = [];
i_Sactq           = [];
WrenchControlledS = [];
dofi              = 1;
if ~Update
    for i=1:Linkage.N %for each link

        VRods_i = Linkage.CVRods{i};

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype == 'S'

            close all
            Linkage.plotq0(i);

            quest  = ['Is the spherical joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Spherical Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Sact  = n_Sact+3;
                    i_Sact  = [i_Sact i];
                    i_Sactq = [i_Sactq dofi dofi+1 dofi+2];

                    quest   = 'Is the spherical joint controlled by wrenches or joint coordinates (qs)?';
                    answer2 = questdlg(quest,'Control',...
                        'Wrench','q','Wrench');

                    switch answer2
                        case 'Wrench'
                            WrenchControlledS = [WrenchControlledS true true true];
                        case 'q'
                            WrenchControlledS = [WrenchControlledS false false false];
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

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='S'&&any(Linkage.i_jact==i)
            
            i_Sactq  = [i_Sactq dofi dofi+1 dofi+2];

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end
end
