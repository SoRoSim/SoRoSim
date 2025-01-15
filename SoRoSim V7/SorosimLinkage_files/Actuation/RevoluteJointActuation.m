%Function that allows the user to specify revolute joint control and actuation
%specifications (29.11.2024)

function [n_Ract,i_Ract,i_Ractq,WrenchControlledR,BqR] = RevoluteJointActuation(S,Update)
if nargin==1
    Update=false;
end
ndof              = S.ndof;
B1                = zeros(ndof,1);

n_Ract            = 0;
i_Ract            = [];
i_Ractq           = [];
BqR               = [];
WrenchControlledR = [];

dofi              = 1;
if ~Update
    for i=1:S.N %for each link

        VRods_i = S.CVRods{i};

        if S.VLinks(S.LinkIndex(i)).jointtype=='R'

            close all
            S.plotq0(i);
            quest  = ['Is the revolute joint of link ',num2str(i),' actuated?'];
            answer = questdlg(quest,'Revolute Joint',...
                'Yes','No','Yes');

            switch answer
                case 'Yes'

                    n_Ract   = n_Ract+1;
                    i_Ract   = [i_Ract i];
                    i_Ractq  = [i_Ractq dofi];
                    B1(dofi) = 1;
                    BqR      = [BqR,B1];
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
        for j = 1:S.VLinks(S.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
else
    for i=1:S.N %for each link

        VRods_i = S.CVRods{i};

        if S.VLinks(S.LinkIndex(i)).jointtype=='R'&&any(S.i_jact==i)
            
            i_Ractq  = [i_Ractq dofi];
            B1(dofi) = 1;
            BqR      = [BqR,B1];
            B1       = zeros(ndof,1);

        end

        dofi = dofi+VRods_i(1).dof;
        for j = 1:S.VLinks(S.LinkIndex(i)).npie-1
            dofi = dofi+VRods_i(j+1).dof;
        end
    end
end

end
