%This toolbox allows the modelling and simulation of static and dynamic
%Run this file to initialize the Rigid-Soft Robotics Toolbox
%Last modified by Anup Teejo Mathew - 18/01/2022

%% Basic operation instructions:
%You can start using this toolbox by defining links, in order to create a link
%define a variable as shown below:

% LinkName = SorosimLink

%You will then have to answer questions in the form of dialougue boxes
%In order to define the link's gemetric and material properties.

%After your link is created you can create a linkage which connects any
%links you've created previously. A linkage is a chain of 1 or more links.
%Syntax for creating a Linkage is given below:

% LinkageName = SorosimLinkage(LinkName1,LinkName2.....LinkNameN)

%Dialougue boxes will then be used to collect input that is used to define
%the linkage properties. You can access any link or linkage properties by calling the property name
%as shown below:

% LinkName.LinkPropertyName
% LinkageName.LinkagePropertyName

%Similarly methods can also be used to evaluate properties outside of the 
%classconstructor after the object is created 

%LinkageName.MethodName(Input1...InputN)

%You can run a static simulation by using the below command:
 
% LinkageName.statics
%The static simulation's output is a vector of joint angles

%A dynamic simulation can be performed using:

%LinkageName.dynamics
%The two outputs of a dynamic simulation are the time vector (t) and a
%matrix containing the configuration (q) and velocity (qd) at every time 
%element of (t). It is a good practice to save the outputs of the dynamic
%simulation for easy access.

%The examples folder of the toolbox contains some saved Linkages and Links
%you can run simulations for 

%%%%%%%%%%%%%%%%%%%%%%%% CLASSES AND THEIR PROPERTIES AND METHODS%%%%%%%%%%%%%%%%%%%%%
%% 1. SorosimLink Class

% %General Properties
        
% jointtype    %Type of joint used to connect the link (lumped DoF). (R) for Revolute,(P) for Prismatic, (H) for Helical, (U) for Universal, (C) for Cylindrical, (A) for Planar, (S) for Spherical, (F) for Free motion and (N) for Fixed
% linktype     %'s' for soft or 'r' for rigid
% npie         %Number of Cosserat rod pieces. For a rigid link, npie = 1 (of the joint). For a soft link, npie=1+number of divisions 
% 
% %Geometric Properties
% 
% ld        %Length of each divisions of the link (soft link) [m]
% L         %Total length of the link [m]
% CS        %Cross section shape: 'R' for rectangular 'C' for circular 'E' for elliptical
% r         %Radius as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
% h         %Height as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
% w         %Width as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
% a         %Semi-major axis as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
% b         %Semi-minor axis as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
% cx        %Local translation in the x direction from the base frame to the CM (only used for plotting, applicable to default rigid bodies)
% gi        %Fixed transformation from joint (X=1) to center of mass for ridig link to center of area for soft link
% gf        %Fixed transformation to the tip from center of mass for ridig link from center of area for soft link
% 
% %Material
% 
% E         %Young's modulus [Pa]
% Poi       %Poisson's ratio [-]
% G         %Shear modulus [Pa]
% Eta       %Material Damping [Pa.s]
% Rho       %Density [kg/m^3]
% Kj        %Joint Stiffness Matrix
% Dj        %Joint Damping Matrix
% 
% M        %Inertia matrix at the CM (only for rigid body)
% 
% %Plot Properties
% 
% n_l       %Number of cross sections per division. (default value: 10)
% n_r       %Number of radial points if the cross section is circular or ellipsoidal (default value: 18)
% color     %Color of link (random by default)
% alpha     %transparency (default: 1 for opaque)
% 
% CPF       %Custom plot function for rigid bodies (logical 1 or 0)
% PlotFn    %Handle of function to plot the geometry (for rigid link)
% Lscale    %Scaling factor for plotting symbols or axes

%%%%Methods%%%%

%% 2. SorosimRod

% Type         %Base type (Monomial, Lagrange Polynomial, Linear Interpolation, Gaussian, Custom, Non-linear Gaussian)
% SubClass     %Now only for FEM Like basis (linear,quadratic,cubic)
% Phi_dof      %(6x1) array specifying the allowable deformation modes of a soft division. 1 if allowed 0 if not.
% Phi_odr      %(6x1) array specifying the order of allowed DoF (0: constant, 1: linear, 2: quadratic,...)
% dof          %degress of freedom of each base
% 
% Phi_h          %Function handle for base
% Phi            %(6xdof) Base matrix calculated at lumped joints or ((6xnGauss)xdof) base matrices computed at every significant points of a soft division
% Phi_Z1         %Base calculated at 4th order first Zanna point (Xs+Z1*(delta(Xs)))
% Phi_Z2         %Base calculated at 4th order second Zanna point (Xs+Z2*(delta(Xs)))
% Phi_Z          %Base calculated at 2nd order Zanna point 
% 
% xi_starfn    %Reference strain vector as a function of X
% xi_star      %(6x1) reference strain vector at the lumped joint or ((6xnGauss)x4) reference strain vectors computed at Gauss quadrature and Zannah collocation points
% 
% Link         %Link associated with this twist only for soft link
% div          %Division associated with this twist
% 
% nip          %number of integration point including boundaries
% Xs           %integration points 
% Ws           %weights of integration point
% 
% Ms           %Inertia matrix of cross-section (6nip x 6) matrix
% Es           %Stiffness matrix (6nip x 6) matrix, for a rigid joint it is joint stiffness matrix in (6x6)
% Gs           %Damping matrix (6nip x 6) matrix, for a rigid joint it is joint damping matrix in (6x6) 
% 
% Xadd         %additional integration points (nx1) vector 

%%%%Methods%%%%
% Updatexi_star(T)
% UpdateMEG(T)
% UpdateAll(T)

%% 3. SorosimLinkage

% %General Properties

% N            %Total number of Links (not total number of rods)
% ndof         %Total number of DOF
% nsig         %Total number of points at which compulations are performed (significant points): N+N_rigid+sum(N_soft_div*nGauss_div)
% nj           %Total number of virtual joints (rigid joints plus virutal joints of all soft rods)
% VLinks       %Vector of all unique links (obtained from user input) VLinks(
% LinkIndex    %(Nx1) array of indices corresponding to each links. ith Link = S.VLinks(LinkIndex(i))
% CVRods       %Cell element of Rod vectors for each link S.CVRods{i}(j) i: Link index, j=1 for joint, j=2 to ndiv+1 for divisions.
% 
% iLpre        %(Nx1) array corresponding to the Link index of the Link to which the ith Link is connected
% g_ini        %(4Nx4) Fixed initial transformation matrices of Links wrt to the tip of its previous link
% Z_order      %Order of Zannah collocation (2, 4, or 6) default value is 4
% OneBasis     %Enable if every rod is governed by the same q vector. Example a POD basis. (logical 0 by default)
% 
% %Closed Loop Joints (Link A is connect to Link B via a closed loop joint)
% 
% nCLj         %Total number of closed loop joints (default 0)
% iCLA         %(nCLjx1)array corresponding to the index of Link A
% iCLB         %(nCLjx1)array corresponding to the index of Link B
% VRodsCLj     %(nCLjx1)array of Rod vectors corresponding to each closed loop joint
% gACLj        %(nCLjx1)cells of fixed transformation from the tip of Link A to the close loop joint
% gBCLj        %(nCLjx1)cells of fixed transformation from the tip of Link B to the close loop joint
% CLprecompute %Struct element which contains pre-computed Phi_p (cell element of constrain basis), i_sigA (array of significant index corresponding to A), i_sigB (array ofsignificant index corresponding to B), and nCLp (total number of constraints)
% T_BS         %Baumgarte stabilization constant. Lower the value stricter the constrain.
% 
% %External Force Properties
% Gravity       %logical 1 if gravity is present, 0 if not
% G             %Value of G
% 
% %Point forces/moments
% np            %Number of point wrenches (default 0)
% LocalWrench   %logical 1 if point force/moment is a follower force (local frame) and 0 if it is wrt global frame
% Fp_loc        %Cell element of Link, Division numbers, and location if along a soft link: [i,j,X]. Corresponding to the point force/moment location (at the end of a soft division/center of mass of rigid link)
% Fp_vec        %Cell element with value of point forces/moments [Mx My Mz Fx Fy Fz]'
% Fp_sig        %Precomputed value of significant point corresponding to point wrenches
% 
% %Custom external force
% CEF          %logical 1 if custom external force is present 0 if not (default value is 0)
% 
% % Underwater Simulation
% UnderWater    %Enable if it is an underwater simulation (default logical 0)
% Rho_water     %Water density by default 1000 kg/m^3
% M_added       %Added mass for fluid simulations (by default zeros(6*nsig,6))
% DL            %Drag-Lift Matrix (by default zeros(6*nsig,6))
% 
% %Actuation Properties
% Actuated      %logical 1 if actuated 0 if not
% nact          %Total number of actuators
% 
% %Joint actuation parameters
% Bj1                 %Pre-computed actuation base for joints with dof = 1
% N_jact              %Total number of links whos joints are actuated
% n_jact              %Total number of joint actuators (eg. 3 for a spherical joint)
% i_jact              %Array of index of links whos joints are actuated
% i_jactq             %(n_jactx1) array of joint coordinate index of all active joints
% WrenchControlled    %(n_jactx1) array of logical 1 if the joint is wrench controlled 0 if joint coordinate controlled
% ActuationPrecompute %struct of precomputed actuation parameters: n_k (total number of constrained DoF), index_q_u, index_q_k, index_u_k, index_u_u
% 
% %Thread-like actuator for soft links
% n_sact            %Number of soft link actuators
% i_sact            %Index of soft actuators present in i th link, jth divisions. Syntax i_sact{i}{j} will give you a row vector with soft actuator index
% dc                %(n_sactxN) cells of local cable position (0, yp, zp) at Gauss quadrature points of all active soft divisions
% dcp               %(n_sactxN) cells of space derivative of the local cable position (0,yp',zp')
% Sdiv              %(n_sactxN) cells of starting division number
% Ediv              %(n_sactxN) cells of ending division  number
% CableFunction     %Struct with cell elements (Cy_fn and Cz_fn) of parameterized functions corresponding to the y and z coodinates of the cable
% 
% %Custom actuation
% CA               %logical 1 if custom actation is present 0 if not (default value: 0)
% CAI              %logical 1 to apply a custom actuator strength (default value: 0)
% 
% %Pre-computed elastic Properties
% K       %Generalized Stiffness matrix
% Damped  %logical 1 if the soft links are elastically damped logical 0 if not (default value is 1)
% D       %Generalized Damping matrix
% 
% CP1     %Custom constant properties of linkage that can be useful
% CP2
% CP3
% 
% %Plotting
% PlotParameters
% 
% %%%PlotParameters is a struct with following elements%%%
% %Lscale                   %Scaling factor for axis
% %CameraPosition           %CameraPosition with respect to origin
% %CameraUpVector           %Orientation of normal
% %CameraTarget             %Target location
% %Light                    %logical 1 if the light is on, 0 if not. (default value: 1)
% %Az_light                 %Light Azimuth wrt camera
% %El_light                 %Light Elevation wrt camera
% %XLim                     %x limit [X_lower_limt X_upper_limit]
% %YLim                     %y limit [Y_lower_limt Y_upper_limit]
% %ZLim                     %z limit [Z_lower_limt Z_upper_limit]
% %FrameRateValue           %FrameRate for dyanmic plot
% %ClosePrevious            %Logical 0 to not close previous image, 1 to close. (default value: 1)
% %CameraRotationSpeed      %For a cinematic rotation of the scene (default 0)
% %VideoResolution          %1 for full screen resolution (higher the better)


%%%%METHODS%%%%

% g       = FwdKinematics(Linkage,q,i,j);           %to get the transformation matrix at every significant points (arranged as column array) i: link, j: division (j=0 for joints)
% J       = Jacobian(Linkage,q,i,j);                %to get the Jacobian at every significant points (arranged as column array)
% Jd      = Jacobiandot(Linkage,q,qd,i,j);          %to get the derivative of Jacobian at every significant points (arranged as column array)
% xi      = ScrewStrain(Linkage,q,i,j)              %to get the screw strain at every significant points (arranged as column array)
% eta     = ScrewVelocity(Linkage,q,qd,i,j);        %to get the screw velocity at every significant points (arranged as column array)
% D       = findD(Linkage);                    %to compute and get the generalized damping matrix
% K       = findK(Linkage)                        %to compute and get the generalized stiffness matrix
% Bq      = ActuationMatrix(Linkage,q);             %to get the generalized actuation matrix (custom actuation not included)
% M       = GeneralizedMassMatrix(Linkage,q,t)        %to get the generalized mass matrix
% C       = GeneralizedCoriolisMatrix(Linkage,q,qd) %to get the generalized coriolis matrix
% F       = GeneralizedExternalForce(Linkage,q,qd)  %to get the generalized external force matrix
% [t,qqd] = dynamics(Linkage,x0,dynamicAction,dynamicsOptions) %for dynamic simulation
% [q,u,lambda] = statics(Linkage,x0,action,staticsOptions) %for static simulation
% 
% plotq0(Linkage,Lh,Dh,CLh);       %to plot the free body diagram of the linkage
% plotq(Linkage,q);              %to plot the state of the linkage for a given q
% plotqt(Linkage,t,qqd);          %to get dynamic simulation video output for a given t (time array) and qqd (array of joint coordinates and their time derivatives)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

%% startup
clc
clear variables

restoredefaultpath
addpath(genpath('Basic_functions'))
addpath('Custom')
addpath('SorosimLink_files')
addpath(genpath('SorosimRod_files'))
addpath(genpath('SorosimLinkage_files')) %include subfolders
addpath('SorosimContact_files')
addpath('noGUI')

% --- locate repo root ---
thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);    % Differentiable_SoRoSim
repoRoot = fileparts(thisDir);     % SoRoSim

% --- iDCOL paths ---
idcolMex  = fullfile(repoRoot,'external','iDCOL','mex');
idcolCore = fullfile(repoRoot,'external','iDCOL','core');

addpath(idcolMex);
addpath(idcolCore);

if exist('.\LinkageProgress.mat','file')
    delete('LinkageProgress.mat')
end
if exist('.\Basis_properties.mat','file')
    delete('Basis_properties.mat')
end
if exist('.\cableactuation.mat','file')
    delete('cableactuation.mat')
end
if exist('.\CablePoints.mat','file')
    delete('CablePoints.mat')
end


basicDir  = fullfile('Basic_functions');

% Try the MEX once; if it errors, rebuild then retry once.
try
    variable_expmap_g_mex(zeros(6,1));
    fprintf('[startup] .mex files OK.\n');
    pause(0.3)
    clc
catch ME1
    fprintf('[startup] MEX call failed: %s\n', ME1.message);
    fprintf('[startup] Running convert2MEX.m...\n');
    wd = pwd;
    try
        cd(basicDir);
        run('convert2MEX.m');
        rehash;  % refresh path
        variable_expmap_g_mex(zeros(6,1));
        fprintf('[startup] Rebuild succeeded. MEX OK.\n');
    catch ME2
        warning('%s', sprintf('[startup] Rebuild failed or MEX still not working: %s', ME2.message));
    end
    cd(pwd)
end

req = struct( ...
  'name', {'Image Processing Toolbox','Optimization Toolbox','Symbolic Math Toolbox'}, ...
  'key',  {'Image_Toolbox','Optimization_Toolbox','Symbolic_Toolbox'}, ...
  'probe',{'imresize','fmincon','sym'});   % representative functions

missing = false(1,numel(req));
for i = 1:numel(req)
    hasLic  = license('test', req(i).key);          % license available?
    hasFunc = exist(req(i).probe, 'file') == 2;     % function on path?
    missing(i) = ~(hasLic && hasFunc);
end

if any(missing)
    fprintf(2,'[startup] Missing required toolboxes:\n');
    for i = find(missing)
        fprintf(2,'  - %s\n', req(i).name);
    end
    fprintf(2,'[startup] Install via Home > Add-Ons > Get Add-Ons, then rerun startup.m.\n');
    return
end

disp('Welcome to SoRoSim Toolbox')
disp('Type LinkName=SorosimLink to create the links (joint and body)')
disp('Type LinkageName=SorosimLinkage(LinkName1,LinkName2,...,LinkNameN) to create linkages by combining links')
disp('For static equilibrium problem type [q,u]=LinkageName.statics')
disp('For dynamics problem type [t,qqd] = LinkageName.dynamics')


