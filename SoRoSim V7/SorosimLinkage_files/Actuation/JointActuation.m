%Function to identify if a joint is actuated either by torque or joint
%coordintates. Outputs are the number of joint actuators, indices of links at
%which the joint is actuated, corresponding joint indices in Qspace, variable to
%save if the joint is wrench controlled or not, and the actuation basis for 1 dof
%joints. 
%Last modified by Anup Teejo Mathew - 09/12/2024

function [n_jact,i_jact,i_jactq,WrenchControlled,Bj1] = JointActuation(Linkage,Update)

    if any(Linkage.jointtype=='R')%revolute joint
        if Update
            [n_Ract,i_Ract,i_Ractq,WrenchControlledR,BR] = RevoluteJointActuation(Linkage,Update);
        else
            [n_Ract,i_Ract,i_Ractq,WrenchControlledR,BR] = RevoluteJointActuation(Linkage);
        end
    else
        n_Ract            = 0;
        i_Ract            = [];
        i_Ractq           = [];
        WrenchControlledR = [];
        BR                = [];
    end

    n_jact           = n_Ract;
    i_jact           = i_Ract;
    i_jactq          = i_Ractq;
    WrenchControlled = WrenchControlledR;
    Bj1              = BR;


    %Prismatic joint
    if any(Linkage.jointtype=='P')
        if Update
            [n_Pact,i_Pact,i_Pactq,WrenchControlledP,BP] = PrismaticJointActuation(Linkage,Update);
        else
            [n_Pact,i_Pact,i_Pactq,WrenchControlledP,BP] = PrismaticJointActuation(Linkage);
        end
    else
        n_Pact            = 0;
        i_Pact            = [];
        i_Pactq           = [];
        WrenchControlledP = [];
        BP                = [];
    end

    n_jact           = n_jact+n_Pact;
    i_jact           = [i_jact i_Pact];
    i_jactq          = [i_jactq i_Pactq];
    WrenchControlled = [WrenchControlled WrenchControlledP];
    Bj1              = [Bj1 BP];

    %Helical joint
    if any(Linkage.jointtype=='H')
        if Update
            [n_Hact,i_Hact,i_Hactq,WrenchControlledH,BH] = HelicalJointActuation(Linkage,Update);
        else
            [n_Hact,i_Hact,i_Hactq,WrenchControlledH,BH] = HelicalJointActuation(Linkage);
        end
    else
        n_Hact            = 0;
        i_Hact            = [];
        i_Hactq           = [];
        WrenchControlledH = [];
        BH                = [];
    end

    n_jact           = n_jact+n_Hact;
    i_jact           = [i_jact i_Hact];
    i_jactq          = [i_jactq i_Hactq];
    WrenchControlled = [WrenchControlled WrenchControlledH];
    Bj1              = [Bj1 BH];

    %Cylindrical joint
    if any(Linkage.jointtype=='C')
        if Update
            [n_Cact,i_Cact,i_Cactq,WrenchControlledC] = CylindricalJointActuation(Linkage,Update);
        else
            [n_Cact,i_Cact,i_Cactq,WrenchControlledC] = CylindricalJointActuation(Linkage);
        end
    else
        n_Cact            = 0;
        i_Cact            = [];
        i_Cactq           = [];
        WrenchControlledC = [];
    end

    n_jact           = n_jact+n_Cact;
    i_jact           = [i_jact i_Cact];
    i_jactq          = [i_jactq i_Cactq];
    WrenchControlled = [WrenchControlled WrenchControlledC];

    %Planar joint 
    if any(Linkage.jointtype=='A')
        if Update
            [n_Aact,i_Aact,i_Aactq,WrenchControlledA] = PlanarJointActuation(Linkage,Update);
        else
            [n_Aact,i_Aact,i_Aactq,WrenchControlledA] = PlanarJointActuation(Linkage);
        end
    else
        n_Aact            = 0;
        i_Aact            = [];
        i_Aactq           = [];
        WrenchControlledA = [];
    end

    n_jact           = n_jact+n_Aact;
    i_jact           = [i_jact i_Aact];
    i_jactq          = [i_jactq i_Aactq];
    WrenchControlled = [WrenchControlled WrenchControlledA];

    %Spherical joint 
    if any(Linkage.jointtype=='S')
        if Update
            [n_Sact,i_Sact,i_Sactq,WrenchControlledS] = SphericalJointActuation(Linkage,Update);
        else
            [n_Sact,i_Sact,i_Sactq,WrenchControlledS] = SphericalJointActuation(Linkage);
        end
    else
        n_Sact            = 0;
        i_Sact            = [];
        i_Sactq           = [];
        WrenchControlledS = [];
    end

    n_jact           = n_jact+n_Sact;
    i_jact           = [i_jact i_Sact];
    i_jactq          = [i_jactq i_Sactq];
    WrenchControlled = [WrenchControlled WrenchControlledS];

    %Free joint 
    if any(Linkage.jointtype=='F')
        if Update
            [n_Fact,i_Fact,i_Factq,WrenchControlledF] = FreeJointActuation(Linkage,Update);
        else
            [n_Fact,i_Fact,i_Factq,WrenchControlledF] = FreeJointActuation(Linkage);
        end
        
    else
        n_Fact            = 0;
        i_Fact            = [];
        i_Factq           = [];
        WrenchControlledF = [];
    end

    n_jact           = n_jact+n_Fact;
    i_jact           = [i_jact i_Fact];
    i_jactq          = [i_jactq i_Factq];
    WrenchControlled = [WrenchControlled WrenchControlledF];
end

