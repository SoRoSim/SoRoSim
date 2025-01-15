%UI figure for link input (Link input, Connection, Location, Orientation)
%Last modified by Anup Teejo Mathew 02.03.2022
function UILinkInput(S,LinkNames,i_Link)

close all

S.plotq0;

fig      = uifigure('Position',[100 100 400 325]);
fig.Name = sprintf('Link %d input',i_Link);

uilabel(fig,'Position',[25 275 350 25],'Text','Link name:');
LinkName = uidropdown(fig,'Position',[150 275 100 22],'Items',LinkNames);

bg  = uibuttongroup(fig,'Position',[25 175 200 30]);
rb1 = uiradiobutton(bg,'Position',[5 5 95 25]);
rb2 = uiradiobutton(bg,'Position',[105 5 95 25]);

rb1.Text = 'Location';
rb2.Text = 'Orientation';

text1 = sprintf('Link %d location (in m) (orientation (cartesian plane angles in rads))',i_Link);
uilabel(fig,'Position',[25 130 350 50],'Text',text1);
l1 = uilabel(fig,'Position',[25 115 350 50],'Text','(With respect to the body frame at the tip of previous Link)');

uilabel(fig,'Position',[30 100 35 25],'Text','x (qx):');
x = uieditfield(fig,'Position',[75 100 50 25],'Value','0');

uilabel(fig,'Position',[130 100 35 25],'Text','y (qy):');
y = uieditfield(fig,'Position',[175 100 50 25],'Value','0');

uilabel(fig,'Position',[230 100 35 25],'Text','z (qz):');
z = uieditfield(fig,'Position',[275 100 50 25],'Value','0');

if S.N>0
    
    uilabel(fig,'Position',[25 225 350 25],'Text','Link number to which this link is connected:');
    uilabel(fig,'Position',[25 210 350 25],'Text','(Previous Link):');
    iList = cell(S.N,1);
    for i=1:S.N+1
        iList{i} = num2str(i-1);
    end
    PreviousLink = uidropdown(fig,'Position',[275 225 100 22],'Items',iList,'Value',num2str(S.N),...
                                  'ValueChangedFcn',@(PreviousLink,event) numberChanged(PreviousLink,l1));
else
    PreviousLink.Value='0';
end

global g_ini_i
g_ini_i=eye(4);

% Create push buttons
uibutton(fig,'push','Position',[75, 50, 50, 25],'Text','Apply',...
                        'ButtonPushedFcn', @(btn,event) ApplyButtonPushed(btn,S,LinkNames,LinkName,PreviousLink,rb1,x,y,z,l1));
uibutton(fig,'push','Position',[175, 50, 50, 25],'Text','Done',...
                       'ButtonPushedFcn', @(btn,event) DoneButtonPushed(btn,LinkNames,LinkName,PreviousLink,rb1,x,y,z));
uibutton(fig,'push','Position',[275, 50, 50, 25],'Text','Reset',...
                        'ButtonPushedFcn', @(btn,event) ResetButtonPushed(btn,S,LinkNames,LinkName,PreviousLink,x,y,z));
% if i_Link>1 % to save progress
% uibutton(fig,'push','Position',[150, 20, 100, 25],'Text','Save Progress',...
%                        'ButtonPushedFcn', @(btn,event) SaveButtonPushed(S));
% end

uiwait(fig)

end

% Create the function for the ButtonPushedFcn callback


% function SaveButtonPushed(S)
%     save('LinkageProgress.mat','S')
% end

function ApplyButtonPushed(btn,S,LinkNames,LinkName,PreviousLink,rb1,x,y,z,l1)

global g_ini_i

if rb1.Value
    xi      = [0;0;0;eval(x.Value);eval(y.Value);eval(z.Value)];
    g_ini_i = g_ini_i*variable_expmap_g(xi);
    x.Value = '0'; y.Value = '0'; z.Value = '0';
else
    xi      = [eval(x.Value);eval(y.Value);eval(z.Value);0;0;0];
    g_ini_i = g_ini_i*variable_expmap_g(xi);
    x.Value = '0'; y.Value = '0'; z.Value = '0';
end

S.N        = S.N+1;
LinkIndex_i = find(strcmp(LinkNames,LinkName.Value));
iLpre_i     = str2double(PreviousLink.Value);


S.LinkIndex = [S.LinkIndex;LinkIndex_i];
S.iLpre     = [S.iLpre;iLpre_i];
S.g_ini     = [S.g_ini;g_ini_i];

Lscale = 0;
for i=1:S.N
    Lscale = Lscale+S.VLinks(S.LinkIndex(i)).L;
end
S.PlotParameters.Lscale = Lscale;

ndof    = S.ndof;
VRods = SorosimRod.empty(S.VLinks(LinkIndex_i).npie,0);

R          = jointRod_pre(S.VLinks(LinkIndex_i),[]); %for each joint and rigid link
VRods(1) = R;
ndof       = ndof+R.dof;

for j=1:S.VLinks(LinkIndex_i).npie-1 %for each of the soft link divisions

    R            = SorosimRod;
    VRods(j+1) = R;

end

S.CVRods = [S.CVRods;{VRods}];
S.ndof     = ndof;

close(1)
S.plotq0;
l1.Text = '(With respect to the body frame at the base of the Link)';

end

function DoneButtonPushed(btn,LinkNames,LinkName,PreviousLink,rb1,x,y,z)

global g_ini_i

if rb1.Value
    xi      = [0;0;0;eval(x.Value);eval(y.Value);eval(z.Value)];
    g_ini_i = g_ini_i*variable_expmap_g(xi);
else
    xi      = [eval(x.Value);eval(y.Value);eval(z.Value);0;0;0];
    g_ini_i = g_ini_i*variable_expmap_g(xi);
end

LinkIndex_i = find(strcmp(LinkNames,LinkName.Value));
iLpre_i     = str2double(PreviousLink.Value);

save('Temp_LinkageAssembly','g_ini_i','LinkIndex_i','iLpre_i')
close all
closereq();
end

function ResetButtonPushed(btn,S,LinkNames,LinkName,PreviousLink,x,y,z)

global g_ini_i
g_ini_i = eye(4);
x.Value = '0'; y.Value = '0'; z.Value = '0';

S.N        = S.N+1;
LinkIndex_i = find(strcmp(LinkNames,LinkName.Value));
iLpre_i     = str2double(PreviousLink.Value);

S.LinkIndex = [S.LinkIndex;LinkIndex_i];
S.iLpre     = [S.iLpre;iLpre_i];
S.g_ini     = [S.g_ini;g_ini_i];

Lscale = 0;
for i=1:S.N
    Lscale = Lscale+S.VLinks(S.LinkIndex(i)).L;
end
S.PlotParameters.Lscale = Lscale;

ndof    = S.ndof;
VRods = SorosimRod.empty(S.VLinks(LinkIndex_i).npie,0);

R          = jointRod_pre(S.VLinks(LinkIndex_i),[]); %for each joint and rigid link
VRods(1) = R;
ndof       = ndof+R.dof;

for j=1:S.VLinks(LinkIndex_i).npie-1 %for each of the soft link divisions

    R          = SorosimRod;
    VRods(j+1) = R;

end

S.CVRods = [S.CVRods;{VRods}];
S.ndof     = ndof;
S.plotq0;
end

function numberChanged(PreviousLink,l1)
l1.Text = '(With respect to the body frame at the tip of previous Link)';

global g_ini_i
g_ini_i = eye(4);
end
