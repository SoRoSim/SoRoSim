%Class to define a SorosimLink
%A SorosimLink consists of a rigid joint and a body.
%Last modified by Anup Teejo Mathew 27.11.2024
classdef SorosimLink < handle %pass by reference (no copy of memory is made)
    
    properties
        
        %General Properties
        
        jointtype    %Type of joint used to connect the link (lumped DoF). (R) for Revolute,(P) for Prismatic, (H) for Helical, (U) for Universal, (C) for Cylindrical, (A) for Planar, (S) for Spherical, (F) for Free motion and (N) for Fixed
        linktype     %'s' for soft or 'r' for rigid
        npie         %Number of Cosserat rod pieces. For a rigid link, npie = 1 (of the joint). For a soft link, npie=1+number of divisions 
               
        %Geometric Properties
        
        ld        %Length of each divisions of the link (soft link) [m]
        L         %Total length of the link [m]
        CS        %Cross section shape: 'R' for rectangular 'C' for circular 'E' for elliptical
        r         %Radius as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
        h         %Height as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
        w         %Width as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
        a         %Semi-major axis as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
        b         %Semi-minor axis as a function of X1 (X1=X/L, X1 varies from 0 to 1) [m]
        cx        %Local translation in the x direction from the base frame to the CM (only used for plotting, applicable to default rigid bodies)
        gi        %Fixed transformation from joint (X=1) to center of mass for ridig link to center of area for soft link
        gf        %Fixed transformation to the tip from center of mass for ridig link from center of area for soft link
        
        %Material
        
        E         %Young's modulus [Pa]
        Poi       %Poisson's ratio [-]
        G         %Shear modulus [Pa]
        Eta       %Material Damping [Pa.s]
        Rho       %Density [kg/m^3]
        Kj        %Joint Stiffness Matrix
        Dj        %Joint Damping Matrix
        
        M        %Inertia matrix at the CM (only for rigid body)
        
        %Plot Properties
        
        n_l       %Number of cross sections per division. (default value: 10)
        n_r       %Number of radial points if the cross section is circular or ellipsoidal (default value: 18)
        color     %Color of link (random by default)
        alpha     %transparency (default: 1 for opaque)
        
        CPF       %Custom plot function for rigid bodies (logical 1 or 0)
        PlotFn    %Handle of function to plot the geometry (for rigid link)
        Lscale    %Scaling factor for plotting symbols or axes
        
    end
    
    
    %%
    methods
        function Link = SorosimLink(varargin)
            if nargin==0
                
                quest = 'Select link type:';
                answ  = questdlg(quest,'Link type',...
                    'Soft','Rigid','Soft');
                
                switch answ
                    case 'Rigid'
                        
                        Link.linktype='r';
                        
                        badanswer = true;
                        
                        while badanswer
                            
                            prompt           = {'Joint type (Enter (R) for Revolute,(P) for Prismatic, (H) for Helical, (C) for Cylindrical, (A) for Planar, (S) for Spherical, (F) for Free motion and (N) for Fixed): ',...
                                                'Cross-section shape (C for circular, R for rectangular, E for elliptical):','Joint stiffness (dof x dof matrix) [Nm/rad or N/m]:',...
                                                'Density [kg/m^3]:','Length of rigid link [m]:'};
                            dlgtitle         = 'Rigid Link Properties';
                            definput         = {'N','C','0','1000','0.3'};
                            opts.Interpreter = 'tex';
                            answer           = inputdlg(prompt,dlgtitle,[1 60],definput,opts);
                            
                            jointtype    = answer{1};
                            CS           = answer{2};
                            Kj           = eval(answer{3});
                            Rho          = eval(answer{4}); 
                            L            = eval(answer{5});
                            
                            badanswer = LinkInputCheck(jointtype,CS,Kj,badanswer);
                            
                            if L<0
                                uiwait(msgbox('In a classical universe, length cannot be negative','Error','error'));
                                badanswer = true;
                            end
                            if ~badanswer
                                
                                Link.jointtype = jointtype;
                                if ~(jointtype=='N')
                                    Link.Kj = Kj;
                                    Link.Dj = 0;%default
                                end
                                Link.npie      = 1;
                                Link.Rho       = Rho;
                                Link.CS        = CS;
                                Link.L         = L;
                            end
                        
                        end
                        if L>0
                            if CS=='C'
    
                                prompt           = {'Initial radius [m]:','Final radius [m]:'};
                                dlgtitle         = 'Circular cross section properties';
                                definput         = {'0.03','0.03'};
                                opts.Interpreter = 'tex';
                                answer           = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
    
                                ri        = eval(answer{1});
                                rf        = eval(answer{2});
    
                                syms X1
                                r   = ri+X1*(rf-ri);
    
                                if has(r,X1)
                                    r = matlabFunction(r);
                                else
                                    r = str2func(['@(X1)' num2str(ri)]);
                                end
    
                                Link.r = r;
    
                                [M,cx] = RigidBodyProperties(CS,L,Rho,r);
    
                            elseif CS=='R'
    
                                prompt           = {'Initial height [m]:','Initial width [m]:','Final height [m]:','Final width [m]:'};
                                dlgtitle         = 'Rectangular cross section properties';
                                definput         = {'0.02','0.02','0.02','0.02'};
                                opts.Interpreter = 'tex';
                                answer           = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
    
                                hi       = eval(answer{1});
                                wi       = eval(answer{2});
                                hf       = eval(answer{3});
                                wf       = eval(answer{4});
    
                                syms X1
                                h   = hi+X1*(hf-hi);
                                w   = wi+X1*(wf-wi);
    
                                if has(h,X1)
                                    h = matlabFunction(h);
                                else
                                    h = str2func(['@(X1)' num2str(hi)]);
                                end
    
                                if has(w,X1)
                                    w = matlabFunction(w);
                                else
                                    w = str2func(['@(X1)' num2str(wi)]);
                                end
    
                                Link.h = h;
                                Link.w = w;
                                
                                [M,cx] = RigidBodyProperties(CS,L,Rho,h,w);
    
                            elseif CS=='E'
    
                                prompt           = {'Initial semi-major axis [m]:','Initial semi-minor axis [m]:','Final semi-major axis [m]:','Final semi-minor axis [m]:'};
                                dlgtitle         = 'Elliptical cross section properties';
                                definput         = {'0.04','0.02','0.04','0.02'};
                                opts.Interpreter = 'tex';
                                answer           = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
    
                                ai       = eval(answer{1});
                                bi       = eval(answer{2});
                                af       = eval(answer{3});
                                bf       = eval(answer{4});
    
                                syms X1
                                a   = ai+X1*(af-ai);
                                b   = bi+X1*(bf-bi);
    
                                if has(a,X1)
                                    a = matlabFunction(a);
                                else
                                    a = str2func(['@(X1)' num2str(ai)]);
                                end
    
                                if has(b,X1)
                                    b = matlabFunction(b);
                                else
                                    b = str2func(['@(X1)' num2str(bi)]);
                                end
    
                                Link.a = a;
                                Link.b = b;
    
                                [M,cx] = RigidBodyProperties(CS,L,Rho,a,b);
                            end        
                        else % L==0
                            M = zeros(6,6);
                            cx = 0;
                        end
    
                        gi = [eye(3),[cx;0;0];[0 0 0 1]];
                        gf = [eye(3),[L-cx;0;0];[0 0 0 1]];
                        Lscale = (M(4,4)/Rho)^(1/3); %scale with cube root of volume
                        Lscale(Lscale==0)=0.1;
                        Link.cx = cx;
                        Link.gi = gi;
                        Link.gf = gf;
                        Link.M = M;
                        
                        
                        %%
                    case 'Soft'
                        
                        Link.linktype = 's';
                        badanswer = true;
                        
                        while badanswer
                            
                            prompt           = {'Joint type (Enter (R) for Revolute,(P) for Prismatic, (H) for Helical, (C) for Cylindrical, (A) for Planar, (S) for Spherical, (F) for Free motion and (N) for Fixed):',...
                                                'Cross-section shape (C for circular, R for rectangular, E for elliptical):','Joint stiffness (matrix) [Nm/rad or N/m]:',...    
                                                'Number of divisions:',...
                                                'Density (kg/m^3):','Youngs Modulus (N/m^2)','Poissons Ratio:','Material Damping (Pa.s)(for dynamic simulation):'};
                            dlgtitle         = 'Soft Link Properties';
    
                            definput         = {'N','C','0','1','1000','1e6','0.5','1e4'};
                            opts.Interpreter = 'tex';
                            answer           = inputdlg(prompt,dlgtitle,[1 60],definput,opts);
    
                            jointtype     = answer{1};
                            CS            = answer{2};
                            Kj            = eval(answer{3});
                            ndiv          = eval(answer{4});
                            Rho           = eval(answer{5});
                            E             = eval(answer{6});
                            Poi           = eval(answer{7});
                            Eta           = eval(answer{8});
                            
                            badanswer = LinkInputCheck(jointtype,CS,Kj,badanswer);
                            
                            if ~badanswer
                                
                                Link.jointtype = jointtype;
                                if jointtype=='N'
                                    Link.Kj = [];
                                else
                                    Link.Kj = Kj;
                                    Link.Dj = 0;
                                end
                                Link.npie      = ndiv+1;
                                Link.CS        = CS;
                                Link.Rho       = Rho;
                                Link.E         = E;
                                Link.Poi       = Poi;
                                Link.Eta       = Eta;
                                G            = E/(2*(1+Poi));
                                Link.G         = G;
                            end
                            
                        end
                        
                        ld   = cell(1,ndiv);
                        r    = cell(1,ndiv);
                        h    = cell(1,ndiv); w  = cell(1,ndiv);
                        a    = cell(1,ndiv); b  = cell(1,ndiv);
                        gi   = cell(1,ndiv); gf = cell(1,ndiv);
                        
                        for j=1:ndiv
                            
                            if CS=='C'
                                
                                prompt           = {'Length (m):','Initial radius (m):','Final radius (m):'};
                                dlgtitle         = ['Enter geometric properties of division ',num2str(j)];
                                definput         = {'0.3','0.03','0.03'};
                                opts.Interpreter = 'tex';
                                answer2          = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                                
                                ldj   = eval(answer2{1});
                                rd_i  = eval(answer2{2});
                                rd_f  = eval(answer2{3});
                                                         
                                syms X1
                                r_sym = rd_i+X1*(rd_f-rd_i);
    
                                if has(r_sym,X1)
                                    r_fn = matlabFunction(r_sym);
                                else
                                    r_fn = str2func(['@(X1)' num2str(rd_i)]);
                                end
    
                                r{j} = r_fn;
                                
                                if j==1
                                    A0 = pi*rd_i^2;
                                end
                                
                            elseif CS=='R'
                                
                                prompt           = {'Length (m):','Initial height (m):','Initial width (m):','Final height (m):','Final width (m):'};
                                dlgtitle         = ['Enter geometric properties of division ',num2str(j)];
                                definput         = {'0.3','0.02','0.02','0.02','0.02'};
                                opts.Interpreter = 'tex';
                                answer2 = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                                
                                ldj    = eval(answer2{1});
                                hi_p   = eval(answer2{2});
                                wi_p   = eval(answer2{3});
                                hf_p   = eval(answer2{4});
                                wf_p   = eval(answer2{5});
                                
                                syms X1
                                h_sym = hi_p+X1*(hf_p-hi_p);
                                w_sym = wi_p+X1*(wf_p-wi_p);
    
                                if has(h_sym,X1)
                                    h_fn = matlabFunction(h_sym);
                                else
                                    h_fn = str2func(['@(X1)' num2str(hi_p)]);
                                end
    
                                if has(w_sym,X1)
                                    w_fn = matlabFunction(w_sym);
                                else
                                    w_fn = str2func(['@(X1)' num2str(wi_p)]);
                                end
                                
                                h{j} = h_fn;
                                w{j} = w_fn;
                                if j==1
                                    A0 = hi_p*wi_p;
                                end
                                                            
                             elseif CS=='E'
                                
                                prompt           = {'Length (m):','Initial semi-major axis (m):','Initial semi-minor axis (m):','Final semi-major axis (m):','Final semi-minor axis (m):'};
                                dlgtitle         = ['Enter geometric properties of division ',num2str(j)];
                                definput         = {'0.3','0.04','0.02','0.04','0.02'};
                                opts.Interpreter = 'tex';
                                answer2 = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                                
                                ldj    = eval(answer2{1});
                                ai_p   = eval(answer2{2});
                                bi_p   = eval(answer2{3});
                                af_p   = eval(answer2{4});
                                bf_p   = eval(answer2{5});
                                
                                syms X1
                                a_sym = ai_p+X1*(af_p-ai_p);
                                b_sym = bi_p+X1*(bf_p-bi_p);
    
                                if has(a_sym,X1)
                                    a_fn = matlabFunction(a_sym);
                                else
                                    a_fn = str2func(['@(X1)' num2str(ai_p)]);
                                end
    
                                if has(b_sym,X1)
                                    b_fn = matlabFunction(b_sym);
                                else
                                    b_fn = str2func(['@(X1)' num2str(bi_p)]);
                                end
                                
                                a{j} = a_fn;
                                b{j} = b_fn;
                                if j==1
                                    A0 = pi*ai_p*bi_p;
                                end
                            end
                            
                            ld{j}     = ldj;
                            gi{j}     = eye(4);
                            gf{j}     = eye(4);
                            
                        end
                        
                        Link.L   = sum(cell2mat(ld));
                        Link.ld  = ld;
                        Link.r   = r;
                        Link.w   = w;
                        Link.h   = h;
                        Link.a   = a;
                        Link.b   = b;
                        Link.gi  = gi;
                        Link.gf  = gf;
                        Lscale = (A0*Link.L).^(1/3);
    
                end
    
                    Link.n_l  = 25;
                    if Link.CS=='R'
                        Link.n_r = 5;
                    else
                        Link.n_r = 18;
                    end
                    Link.color  = rand(1,3);
                    Link.alpha  = 1;
                    Link.CPF    = false;
                    Link.PlotFn = @(g) CustomShapePlot(g);
                    Link.Lscale = Lscale;
                    plotLink(Link)
            else
                if strcmp(varargin{1}, 'empty')
                    disp('Creating empty SorosimLink');
                end
            end
        end
    end

    events
        PropertyChanged
    end
    
    %% static method to create an empty class element
    methods (Static)
        function obj = createEmpty()
            obj = SorosimLink; % Create a default object
            % Optionally clear or reset properties to create an "empty-like" state
        end
    end

    methods
        
        %% Set and property update functions
        
        %Changing r:
        function set.r(SorosimLink, value)
            SorosimLink.r = value;
            SorosimLink.Update()
            notify(SorosimLink, 'PropertyChanged');% Trigger the PropertyChanged event
        end
        
        %Changing w:
        function set.w(SorosimLink, value)
            SorosimLink.w = value;
            SorosimLink.Update()
            notify(SorosimLink, 'PropertyChanged');
        end

        %Changing h:
        function set.h(SorosimLink, value)
            SorosimLink.h = value;
            SorosimLink.Update()
            notify(SorosimLink, 'PropertyChanged');
        end
        
        %Changing a:
        function set.a(SorosimLink, value)
            SorosimLink.a = value;
            SorosimLink.Update()
            notify(SorosimLink, 'PropertyChanged');
        end

        %Changing b:
        function set.b(SorosimLink, value)
            SorosimLink.b = value;
            SorosimLink.Update()
            notify(SorosimLink, 'PropertyChanged');
        end
        
        %Changing L: (rigid or constant CS soft)
        function set.L(SorosimLink, value)
            SorosimLink.L = value;
            SorosimLink.Update() %no need to notify event?
        end
        
        %Changing Rho:
        function set.Rho(SorosimLink, value)
            SorosimLink.Rho = value;
            SorosimLink.Update()
            notify(SorosimLink, 'PropertyChanged');
        end
        
        %Changing E:
        function set.E(SorosimLink, value)
            SorosimLink.E = value;
            SorosimLink.UpdateG();
            notify(SorosimLink, 'PropertyChanged');
        end
        
        %Changing Poi:
        function set.Poi(SorosimLink, value)
            SorosimLink.Poi = value;
            SorosimLink.UpdateG();
            notify(SorosimLink, 'PropertyChanged');
        end
        
        %Changing ld:
        function set.ld(SorosimLink, value)
            SorosimLink.ld = value;
            SorosimLink.UpdateL();
            notify(SorosimLink, 'PropertyChanged');
        end
        
        function set.jointtype(SorosimLink, value)
            SorosimLink.jointtype = value;
            SorosimLink.UpdateKD();
            notify(SorosimLink, 'PropertyChanged');
        end

        function set.Kj(SorosimLink, value)
            SorosimLink.Kj = value;
            notify(SorosimLink, 'PropertyChanged');
        end

        function set.Dj(SorosimLink, value)
            SorosimLink.Dj = value;
            notify(SorosimLink, 'PropertyChanged');
        end
%%        
        %Updating the properties
        function Update(SorosimLink)
            if isempty(SorosimLink.color)
                return
            end
            if SorosimLink.linktype=='r'
                UpdateRigidLink(SorosimLink);
            end
        end
        
        %Update E or Poi
        function UpdateG(SorosimLink)
            if isempty(SorosimLink.color)
                return
            end
            SorosimLink.G=SorosimLink.E/(2*(1+SorosimLink.Poi));
        end
        
        %Update ld
        function UpdateL(SorosimLink)
            if isempty(SorosimLink.color)
                return
            end
            SorosimLink.L = sum(cell2mat(SorosimLink.ld));
        end
        
        function UpdateKD(SorosimLink)
            if isempty(SorosimLink.color)
                return
            end
            if SorosimLink.jointtype~='N'
                SorosimLink.Kj = 0;
                SorosimLink.Dj = 0;
            else
                SorosimLink.Kj = [];
                SorosimLink.Dj = [];
            end

        end
        
        
    end
    
end