%Function that allows the user to specify revolute joint control and actuation
%specifications (29.11.2024)

function [n_Ract,i_Ract,i_Ractq,WrenchControlledR,BR] = RevoluteJointActuation(Linkage,Update)
if nargin==1
    Update=false;
end
ndof              = Linkage.ndof;
B1                = zeros(ndof,1);

n_Ract            = 0;
i_Ract            = [];
i_Ractq           = [];
BR                = [];
WrenchControlledR = [];

dofi              = 1;
if ~Update
    for i=1:Linkage.N %for each link

        VRods_i = Linkage.CVRods{i};

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='R'

            close all
            Linkage.plotq0(i);
            quest  = ['Is the revolute joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Revolute Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Ract   = n_Ract+1;
                    i_Ract   = [i_Ract i];
                    i_Ractq  = [i_Ractq dofi];
                    B1(dofi) = 1;
                    BR       = [BR,B1];
                    B1       = zeros(ndof,1);

                    quest    = 'Is the revolute joint controlled by torque or angle?';
                    answer2  = questdlg(quest,'Control',...
                        'Torque','Angle','Torque');

                    switch answer2
                        case 'Torque'
                            WrenchControlledR = [WrenchControlledR true];
                        case 'Angle'
                            WrenchControlledR = [WrenchControlledR false];
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

        if Linkage.VLinks(Linkage.LinkIndex(i)).jointtype=='R'&&any(Linkage.i_jact==i)
            
            i_Ractq  = [i_Ractq dofi];
            B1(dofi) = 1;
            BR       = [BR,B1];
            B1       = zeros(ndof,1);

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
