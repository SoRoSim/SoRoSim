%UI figure for closed loop input (Link input, Connection, Location, Orientation)
%Last modified by Anup Teejo Mathew 29.11.2024
function UIClosedLoop(Linkage,iCL)

close all

Linkage.plotq0;

fig      = uifigure('Position',[100 100 400 250]);
fig.Name = ['Closed loop joint ',num2str(iCL)];

uilabel(fig,'Position',[25 230 350 25],'Text','Tip of Link A will be connected to tip of Link B');
uilabel(fig,'Position',[25 215 350 25],'Text','(Joint will be defined wrt to the body frame of Link A)');

iList = cell(Linkage.N+1,1);
for i=1:Linkage.N+1
    iList{i} = num2str(i-1);
end

uilabel(fig,'Position',[25 170 350 25],'Text','Link number corresponding to Link A:');
LinkA = uidropdown(fig,'Position',[230 170 100 22],'Items',iList,'Value','1');
uilabel(fig,'Position',[25 130 350 25],'Text','Link number corresponding to Link B:');
LinkB = uidropdown(fig,'Position',[230 130 100 22],'Items',iList);
uilabel(fig,'Position',[25 80 350 25],'Text','Closed loop joint type:');
CLjAB = uidropdown(fig,'Position',[150 80 100 22],'Items',{'Revolute','Prismatic','Helical','Cylindrical','Planar','Spherical','Fixed'});
% Create push buttons
uibutton(fig,'push','Position',[175, 40, 50, 25],'Text','Done',...
                       'ButtonPushedFcn', @(btn,event) DoneButtonPushed(btn,Linkage,iCL,LinkA,LinkB,CLjAB));
uiwait(fig)

end

function DoneButtonPushed(~,Linkage,iCL,LinkA,LinkB,CLjAB)
    
    if isequal(LinkA.Value,LinkB.Value)
        uiwait(msgbox('Link A and B are same','Error','error'))
    else
        Linkage.nCLj      = Linkage.nCLj+1;
        Linkage.iCLA(iCL) = str2double(LinkA.Value);
        Linkage.iCLB(iCL) = str2double(LinkB.Value);

        L = SorosimLink(0);
        switch CLjAB.Value
            case 'Revolute'
                L.jointtype='R';
            case 'Prismatic'
                L.jointtype='P';
            case 'Helical'
                L.jointtype='H';
            case 'Cylindrical'
                L.jointtype='C';
            case 'Planar'
                L.jointtype='A';
            case 'Spherical'
                L.jointtype='S';
            case 'Fixed'
                L.jointtype='N';
        end
        
        iA = str2double(LinkA.Value);
        iB = str2double(LinkB.Value);
        g  = Linkage.FwdKinematics(zeros(Linkage.ndof,1));
        
        i_sigA = 0;
        for i=1:iA
            i_sigA = i_sigA+1;
            if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
                i_sigA = i_sigA+1;
            end
            for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
                i_sigA = i_sigA+Linkage.CVRods{i}(j+1).nip;
            end
        end
        
        i_sigB = 0;
        for i=1:iB
            i_sigB = i_sigB+1;
            if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
                i_sigB = i_sigB+1;
            end
            for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
                i_sigB = i_sigB++Linkage.CVRods{i}(j+1).nip;
            end
        end
           
        if iA==0
            gBtip = g((i_sigB-1)*4+1:i_sigB*4,:);
            if Linkage.VLinks(Linkage.LinkIndex(iB)).linktype=='r'
                gBtip = gBtip*Linkage.VLinks(Linkage.LinkIndex(iB)).gf;
            else
                gBtip = gBtip*Linkage.VLinks(Linkage.LinkIndex(iB)).gf{end};
            end
            
            gACLj = [eye(3),gBtip(1:3,4);[0 0 0 1]]; %only position
            gBCLj = ginv(gBtip)*gACLj;
        elseif iB==0
            gACLj = eye(4);
            gAtip = g((i_sigA-1)*4+1:i_sigA*4,:);
            if Linkage.VLinks(Linkage.LinkIndex(iA)).linktype=='r'
                gAtip = gAtip*Linkage.VLinks(Linkage.LinkIndex(iA)).gf;
            else
                gAtip = gAtip*Linkage.VLinks(Linkage.LinkIndex(iA)).gf{end};
            end
            gBCLj = gAtip;
        else
            gACLj = eye(4);
            gAtip = g((i_sigA-1)*4+1:i_sigA*4,:);
            if Linkage.VLinks(Linkage.LinkIndex(iA)).linktype=='r'
                gAtip = gAtip*Linkage.VLinks(Linkage.LinkIndex(iA)).gf;
            else
                gAtip = gAtip*Linkage.VLinks(Linkage.LinkIndex(iA)).gf{end};
            end
            gBtip = g((i_sigB-1)*4+1:i_sigB*4,:);
            if Linkage.VLinks(Linkage.LinkIndex(iB)).linktype=='r'
                gBtip = gBtip*Linkage.VLinks(Linkage.LinkIndex(iB)).gf;
            else
                gBtip = gBtip*Linkage.VLinks(Linkage.LinkIndex(iB)).gf{end};
            end
            gBCLj = ginv(gBtip)*gAtip;
        end
        
        Linkage.gACLj{iCL} = gACLj;
        Linkage.gBCLj{iCL} = gBCLj;
        
        RodAB = jointRod(L,iCL);
        
        save('Temp_LinkageClosedJoint','iA','iB','RodAB','gACLj','gBCLj')
        close all
        closereq(); 
    end
end
