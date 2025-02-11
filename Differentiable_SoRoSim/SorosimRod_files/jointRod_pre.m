%Function to Pre assign Twist class (basis, dof, etc.) to joints to be
%changed during Linkage creation
%Last modified by Anup Teejo Mathew 30/05/2021

function R = jointRod_pre(L,i) %default

    switch L.jointtype
        case 'R' %if revolute

            J_axis = 'z';
            if J_axis=='x'
                Phi = [1 0 0 0 0 0]';
            elseif J_axis=='y'
                Phi = [0 1 0 0 0 0]';
            else
                Phi = [0 0 1 0 0 0]';
            end

            R = SorosimRod(L,Phi);

        case 'P' %if prismatic
            
            J_axis = 'x';
            if J_axis=='x'
                Phi = [0 0 0 1 0 0]';
            elseif J_axis== 'y'
                Phi = [0 0 0 0 1 0]';
            else
                Phi = [0 0 0 0 0 1]';
            end

            R = SorosimRod(L,Phi);

        case 'H' %if helical
            
            p      = 0.1;
            J_axis = 'x';
            if J_axis=='x'
                Phi = [1 0 0 p 0 0]';
            elseif J_axis== 'y'
                Phi = [0 1 0 0 p 0]';
            else
                Phi = [0 0 1 0 0 p]';
            end

            R = SorosimRod(L,Phi);
            

        case 'C' %if cylinderical
            
            J_axis1 = 'y';
            if J_axis1=='x'
                Phi = [1 0 0 0 0 0;0 0 0 1 0 0]'; %correct this later
            elseif J_axis1== 'y'
                Phi = [0 1 0 0 0 0;0 0 0 0 1 0]';
            else
                Phi = [0 0 1 0 0 0;0 0 0 0 0 1]';
            end

            R = SorosimRod(L,Phi);

        case 'A' %if planar

            J_axis = 'xy';
            if all(J_axis=='xy')
                Phi = [0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0]';
            elseif all(J_axis== 'yz')
                Phi = [1 0 0 0 0 0;0 0 0 0 1 0;0 0 0 0 0 1]';
            else
                Phi = [0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]';
            end

            R = SorosimRod(L,Phi);

        case 'S' %if spherical
            
            Phi = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0]';

            R     = SorosimRod(L,Phi);

        case 'F' %if free motion

            Phi = eye(6);
            R   = SorosimRod(L,Phi);
            
        case 'N'
            
            Phi = [];
            R   = SorosimRod(L,Phi);
            
        otherwise
            error(['Incorrect joint type. Check the joint type of the link ',num2str(i)])
    end
    
end

