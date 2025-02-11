%Function to Assign Twist class (basis, dof, etc.) to joints
%Last modified by Anup Teejo Mathew 29.11.2024

function R = jointRod(L,i) 

    switch L.jointtype
        case 'R' %if revolute
            
            list  = {'x', 'y', 'z'};
            J_axis = listdlg('PromptString',{['Revolute joint of link ',num2str(i)],'Axis of rotation (local):'},...
                             'SelectionMode','single','ListSize',[160 160],'ListString',list,'InitialValue',3);
            if J_axis==1
                Phi = [1 0 0 0 0 0]';
            elseif J_axis==2
                Phi = [0 1 0 0 0 0]';
            else
                Phi = [0 0 1 0 0 0]';
            end

            R        = SorosimRod(L,Phi);

        case 'P' %if prismatic
            
            list  = {'x', 'y', 'z'};
            J_axis = listdlg('PromptString',{['Prismatic joint of link ',num2str(i)],'Axis of translation (local):'},...
                             'SelectionMode','single','ListSize',[160 160],'ListString',list,'InitialValue',3);
            if J_axis==1
                Phi = [0 0 0 1 0 0]';
            elseif J_axis==2
                Phi = [0 0 0 0 1 0]';
            else
                Phi = [0 0 0 0 0 1]';
            end

            R        = SorosimRod(L,Phi);

        case 'H' %if helical
            
            badans = 1;
            
            while badans
                badans=0;
            
                prompt           = {'Joint pitch (m/rad):','Axis of motion (x, y or z) (local):'};
                dlgtitle         = ['Helical joint of link ',num2str(i)];
                definput         = {'0.1','x'};
                opts.Interpreter = 'tex';
                opts.WindowStyle = 'Normal';
                H_ans            = inputdlg(prompt,dlgtitle,[1 75],definput,opts);

                p      = str2double(H_ans{1});
                J_axis = H_ans{2};
                if J_axis=='x'
                    Phi = [1 0 0 p 0 0]';
                elseif J_axis== 'y'
                    Phi = [0 1 0 0 p 0]';
                elseif J_axis== 'z'
                    Phi = [0 0 1 0 0 p]';
                else
                    uiwait(msgbox('Incorrect entry of axis','Error','error'));
                    badans=1;
                end
            end

            R        = SorosimRod(L,Phi);
            

        case 'C' %if cylinderical
            
            list  = {'x', 'y', 'z'};
            J_axis = listdlg('PromptString',{['Cylinderical joint of link ',num2str(i)],'Axis of motion (local):'},...
                             'SelectionMode','single','ListSize',[160 160],'ListString',list,'InitialValue',2);
                
            if J_axis==1
                Phi = [1 0 0 0 0 0;0 0 0 1 0 0]'; %correct this later
            elseif J_axis== 2
                Phi = [0 1 0 0 0 0;0 0 0 0 1 0]';
            else
                Phi = [0 0 1 0 0 0;0 0 0 0 0 1]';
            end

            R        = SorosimRod(L,Phi);

        case 'A' %if planar

            
            list  = {'xy', 'yz', 'xz'};
            J_axis = listdlg('PromptString',{['Planar joint of link ',num2str(i)],'Plane of motion (local):'},...
                             'SelectionMode','single','ListSize',[160 160],'ListString',list,'InitialValue',1);

            if J_axis==1
                Phi = [0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0]';
            elseif J_axis==2
                Phi = [1 0 0 0 0 0;0 0 0 0 1 0;0 0 0 0 0 1]';
            else
                Phi = [0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]';
            end

            R     = SorosimRod(L,Phi);

        case 'S' %if spherical
            
            Phi = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0]';

            R        = SorosimRod(L,Phi);

        case 'F' %if free motion

            Phi=eye(6);

            R        = SorosimRod(L,Phi);
            
        case 'N'
            
            Phi     = [];
            R     = SorosimRod(L,Phi);
            
        otherwise
            error(['Incorrect joint type. Check the joint type of the link ',num2str(i)])
    end
    
end

