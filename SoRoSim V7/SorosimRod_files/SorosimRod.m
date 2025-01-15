%Class that assigns properties of a Cosserat Rod
%Last modified by Anup Teejo Mathew 28.11.2024

classdef SorosimRod < handle %pass by reference (no copy of memory is made)
    
    properties
        Type         %Base type (Monomial, Lagrange Polynomial, Linear Interpolation, Gaussian, Custom, Non-linear Gaussian)
        SubClass     %Now only for FEM Like basis (linear,quadratic,cubic)
        Phi_dof      %(6x1) array specifying the allowable deformation modes of a soft division. 1 if allowed 0 if not.
        Phi_odr      %(6x1) array specifying the order of allowed DoF (0: constant, 1: linear, 2: quadratic,...)
        dof          %degress of freedom of each base
        
        Phi_h          %Function handle for base
        Phi            %(6xdof) Base matrix calculated at lumped joints or ((6xnGauss)xdof) base matrices computed at every significant points of a soft division
        Phi_Z1         %Base calculated at 4th order first Zanna point (Xs+Z1*(delta(Xs)))
        Phi_Z2         %Base calculated at 4th order second Zanna point (Xs+Z2*(delta(Xs)))
        Phi_Z          %Base calculated at 2nd order Zanna point 
        
        xi_starfn    %Reference strain vector as a function of X
        xi_star      %(6x1) reference strain vector at the lumped joint or ((6xnGauss)x4) reference strain vectors computed at Gauss quadrature and Zannah collocation points
        
        Link         %Link associated with this twist only for soft link
        div          %Division associated with this twist

        nip          %number of integration point including boundaries
        Xs           %integration points 
        Ws           %weights of integration point

        Ms           %Inertia matrix of cross-section (6nip x 6) matrix
        Es           %Stiffness matrix (6nip x 6) matrix, for a rigid joint it is joint stiffness matrix in (6x6)
        Gs           %Damping matrix (6nip x 6) matrix, for a rigid joint it is joint damping matrix in (6x6) 
        
        Xadd         %additional integration points (nx1) vector 
    end
    
    methods
        
        function Rod = SorosimRod(varargin)
            
            if nargin==3 %soft piece

                i = varargin{1};
                j = varargin{2};
                Link = varargin{3};
                
                Rod.Link = Link;
                % Attach a listener to the PropertyChanged event of the Link
                addlistener(Rod.Link, 'PropertyChanged', @(src, event) Rod.Link2RodUpdate());
                Rod.div = j;
                
                Z1     = 1/2-sqrt(3)/6;          %Zanna quadrature coefficient 4th order
                Z2     = 1/2+sqrt(3)/6;          %Zanna quadrature coefficient 4th order
                Z      = 1/2;                    %Zanna quadrature coefficient 2nd order
               
                basedef(i,j);
                load('Base_properties.mat','Type','Phi_dof','Phi_odr','xi_stars','SubClass')
                
                Rod.Type = Type;
                Rod.Phi_dof = Phi_dof;
                Rod.Phi_odr = Phi_odr;
%% Compute/Input number of Gauss quadrature point for spatial integration and DOF of the Rod
                
                nGauss_min = 5; %minimum number of Gauss quadrature point per rod

                if any(strcmp(Type,{'Monomial','Legendre Polynomial','Chebychev','Fourier','Custom Basis'})) % all global basis
                        
                    if strcmp(Type,'Fourier')
                        nGauss = max(Phi_odr)*4; % need more
                        dof = sum(Phi_dof.*(2*Phi_odr+1));
                    else
                        nGauss = max(Phi_odr)+1;
                        dof = sum(Phi_dof.*(Phi_odr+1));
                    end

                    nGauss(nGauss<5) = nGauss_min;
                    prompt           = ['Number of Gaussian Quadrature points (min ',num2str(nGauss),'):'];
                    dlgtitle         = 'Gaussian Points ';
                    definput         = {num2str(nGauss)};
                    opts.Interpreter = 'tex';
                    answer2 = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                    nGauss  = str2double(answer2{1});
                    [Xs,Ws,nip] = GaussQuadrature(nGauss);

                else  % all local (element) basis
                    
                    n_ele = max(Phi_odr); % All numbers must be same max is used considering user error
                    nGauss = nGauss_min;
                    
                    if nGauss/n_ele-floor(nGauss/n_ele)==0
                        nGausse = floor(nGauss/n_ele);
                    else
                        nGausse = floor(nGauss/n_ele)+1;
                    end

                    switch Type
                        case 'FEM Like'
                            if strcmp(SubClass,'Linear')
                                nGausse(nGausse<2) = 2; % Minimum nGauss per element
                                dof = sum(Phi_dof*(n_ele+1));
                            elseif strcmp(SubClass,'Quadratic')
                                nGausse(nGausse<3) = 3; % Minimum nGauss per element
                                dof = sum(Phi_dof*(2*n_ele+1));
                            elseif strcmp(SubClass,'Cubic')
                                nGausse(nGausse<4) = 4; % Minimum nGauss per element
                                dof = sum(Phi_dof*(3*n_ele+1));
                            end
                            
                        case {'Gaussian Element', 'Inverse Multi-quadratic'}
                            nGausse(nGausse<3) = 3; % Minimum nGauss per element
                            dof = sum(Phi_dof*(n_ele+1));
                            
                        case 'Hermite Spline'
                            nGausse(nGausse<4) = 4; % Minimum nGauss per element
                            dof = sum(Phi_dof*(2*n_ele+2));
                    end
                    
                    prompt           = ['Gauss Quadrature points per element (min ', num2str(nGausse),'):'];
                    dlgtitle         = 'Gaussian Points ';
                    definput         = {num2str(nGausse)};
                    opts.Interpreter = 'tex';
                    answer2 = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                    nGausse = str2double(answer2{1});
                    
                    [Xse,Wse,nipe]=GaussQuadrature(nGausse); %per element

                    nip = n_ele*(nipe-1)+1;
                    Ws = zeros(nip,1);
                    Xs = zeros(nip,1);
                    Ws(1:end-1)=repmat(Wse(1:end-1),n_ele,1)/n_ele;
                    
                    X0 = 0;
                    for i=1:n_ele
                        Xs((i-1)*(nipe-1)+1:i*(nipe-1)) = X0+Xse(1:nipe-1)/n_ele;
                        X0 = X0+1/n_ele;
                    end
                    Xs(end)=1;

                end
%% Precomputation

                switch Type
                    case 'Monomial'
                        filename = 'Phi_Monomial'; 
                    case 'Legendre Polynomial'
                        filename = 'Phi_LegendrePolynomial';
                    case 'Chebychev'
                        filename = 'Phi_Chebychev';
                    case 'Fourier'
                        filename = 'Phi_Fourier';
                    case 'FEM Like'
                        filename = 'Phi_FEMLike'; 
                        Rod.SubClass = SubClass;
                    case 'Hermite Spline'
                        filename = 'Phi_HermiteSpline';
                    case 'Gaussian Element'
                        filename = 'Phi_Gaussian';
                    case 'Inverse Multi-quadratic'
                        filename = 'Phi_IMQ';
                    case 'Custom Basis'
                        %select function, find dof
                        filename = 'Phi_Custom';
                        Phi_h  = str2func(filename);
                        Phi    = Phi_h(0);
                        dof  = size(Phi,2);
                end

                Phi    = zeros(nip*6,dof); % 6xndof matrix at every Gaussian points
                Phi_Z1 = zeros((nip-1)*6,dof); % 6xndof matrix at every Z1 points (Zannah order 4)
                Phi_Z2 = zeros((nip-1)*6,dof); % 6xndof matrix at every Z2 points (Zannah order 4)
                Phi_Z  = zeros((nip-1)*6,dof); % 6xndof matrix at every Z points (Zannah order 2)

                if strcmp(Type, 'FEM Like')
                    Phi_h = str2func(['@(X,Bdof,Bodr,SubClass)', filename, '(X,Bdof,Bodr,SubClass)']);
                    extra_args = {Phi_dof, Phi_odr, SubClass};
                elseif strcmp(Type, 'Custom Basis')
                    extra_args = {};
                else
                    Phi_h = str2func(['@(X,Bdof,Bodr)', filename, '(X,Bdof,Bodr)']);
                    extra_args = {Phi_dof, Phi_odr};
                end
                
                ld = Link.ld{j};
                Phi_Scale = diag([1/ld 1/ld 1/ld 1 1 1]);

                X = Xs(1);
                Phi(1:6, :) = Phi_Scale*Phi_h(X, extra_args{:});
                for ii = 2:nip
                    X = Xs(ii);
                    Phi(6*(ii-1)+1:6*ii,:) = Phi_Scale*Phi_h(X,extra_args{:});
                    X = Xs(ii-1)+Z1*(Xs(ii)-Xs(ii-1));
                    Phi_Z1(6*(ii-2)+1:6*(ii-1),:) = Phi_Scale*Phi_h(X,extra_args{:});
                    X = Xs(ii-1)+Z2*(Xs(ii)-Xs(ii-1));
                    Phi_Z2(6*(ii-2)+1:6*(ii-1),:) = Phi_Scale*Phi_h(X,extra_args{:});
                    X = Xs(ii-1)+Z*(Xs(ii)-Xs(ii-1));
                    Phi_Z(6*(ii-2)+1:6*(ii-1),:) = Phi_Scale*Phi_h(X,extra_args{:});
                end
                
                % Reference strain precomputation

                syms X;
                xi_stars1 = str2sym(xi_stars{1});
                xi_stars2 = str2sym(xi_stars{2});
                xi_stars3 = str2sym(xi_stars{3});
                xi_stars4 = str2sym(xi_stars{4});
                xi_stars5 = str2sym(xi_stars{5});
                xi_stars6 = str2sym(xi_stars{6});
                xi_stars  = [xi_stars1;xi_stars2;xi_stars3;xi_stars4;xi_stars5;xi_stars6];

                if any(has(xi_stars,X))
                    xi_starfn = matlabFunction(xi_stars);
                else 
                    xi_starfn = str2func(['@(X) [' num2str(double(xi_stars')) ']''']);
                end

                xi_star        = zeros(6*nip,4); % precomputation at all gauss and zannah gauss points
                xi_star(1:6,1) = xi_starfn(Xs(1)); %1
                for ii=2:nip
                    xi_star((ii-1)*6+1:ii*6,1)     = xi_starfn(Xs(ii)); %2 to nGauss
                    xi_star((ii-2)*6+1:(ii-1)*6,2) = xi_starfn(Xs(ii-1)+Z1*(Xs(ii)-Xs(ii-1))); %1 to nGauss-1
                    xi_star((ii-2)*6+1:(ii-1)*6,3) = xi_starfn(Xs(ii-1)+Z2*(Xs(ii)-Xs(ii-1))); %1 to nGauss-1
                    xi_star((ii-2)*6+1:(ii-1)*6,4) = xi_starfn(Xs(ii-1)+Z*(Xs(ii)-Xs(ii-1))); %1 to nGauss-1
                end

                % Precomputation of Cross Sectional Inertia, Elastic, and Damping Matrices               
                [Ms,Es,Gs] = MEG(Link,j,Xs);
%% Property assignment

                Rod.xi_starfn = xi_starfn;
                Rod.xi_star = xi_star;
                Rod.nip = nip;
                Rod.Xs = Xs;
                Rod.Ws = ld*Ws;
                Rod.Ms = Ms;
                Rod.Es = Es;
                Rod.Gs = Gs;
                Rod.Phi    = Phi;
                Rod.Phi_Z1 = Phi_Z1;
                Rod.Phi_Z2 = Phi_Z2;
                Rod.Phi_Z  = Phi_Z;
                Rod.Phi_h   = Phi_h;
                Rod.dof  = dof;

            elseif nargin==2 %joint
                
                Link = varargin{1};
                Phi  = varargin{2};
                dof  = size(Phi,2);

                if isequal(size(Link.Kj),[dof,dof])
                    Rod.Es = Phi*Link.Kj*Phi'; %converting to 6x6 matrix
                else
                    Rod.Es = zeros(6,6);
                end

                if isequal(size(Link.Dj),[dof,dof])
                    Rod.Gs = Phi*Link.Dj*Phi'; %converting to 6x6 matrix
                else
                    Rod.Gs = zeros(6,6);
                end

                Rod.Link = Link;
                Rod.Phi  = Phi;
                Rod.dof  = dof;

                Rod.xi_star = zeros(6,1);

            elseif nargin==0
                
                xi_starfn = str2func('@(X) [0 0 0 1 0 0]''');
                Rod.Phi = [];
                Rod.xi_starfn = xi_starfn;
                Rod.Phi_odr = zeros(6,1);
                Rod.Phi_dof = zeros(6,1);
                Rod.dof = 0;
                
            end
            
        end
    end
        
    methods
%% Update Basis function handle

        function UpdatePhi_h(R) % change integration scheme
            if isempty(R.dof)
                return
            end

            typeToFileMap = containers.Map(...
                {'Monomial', 'Legendre Polynomial', 'Chebychev', 'Fourier', ...
                 'FEM Like', 'Hermite Spline', 'Gaussian Element', ...
                 'Inverse Multi-quadratic', 'Custom Basis'}, ...
                {'Phi_Monomial', 'Phi_LegendrePolynomial', 'Phi_Chebychev', 'Phi_Fourier', ...
                 'Phi_FEMLike', 'Phi_HermiteSpline', 'Phi_Gaussian', ...
                 'Phi_IMQ', 'Phi_Custom'});
            
            % Access the file name based on T.Type
            if isKey(typeToFileMap, R.Type)
                filename = typeToFileMap(R.Type);
            
                % Define T.Phi_h based on type
                if strcmp(R.Type, 'FEM Like')
                    R.Phi_h = str2func(['@(X,Bdof,Bodr,SubClass)', filename, '(X,Bdof,Bodr,SubClass)']);
                elseif strcmp(R.Type, 'Custom Basis')
                    R.Phi_h = str2func(filename); % No function wrapper needed
                else
                    R.Phi_h = str2func(['@(X,Bdof,Bodr)', filename, '(X,Bdof,Bodr)']);
                end
            else
                error('Unknown Type: %s', R.Type);
            end

        end
        
%% Update Quadrature Points and Weights

function UpdateIntegration(R,varargin) %user input is nGauss or nGausse depending on type of basis, if no input is provided it asks

             nGauss_min = 5; %minimum number of Gauss quadrature point per rod

            if any(strcmp(R.Type,{'Monomial','Legendre Polynomial','Chebychev','Fourier','Custom Basis'})) % all global basis
                
                if nargin==1
                    if strcmp(R.Type,'Fourier')
                        nGauss = max(R.Phi_odr)*4; % need more
                    else
                        nGauss = max(R.Phi_odr)+1;
                    end
    
                    nGauss(nGauss<5) = nGauss_min;
                    prompt           = ['Number of Gaussian Quadrature points (min ',num2str(nGauss),'):'];
                    dlgtitle         = 'Gaussian Points ';
                    definput         = {num2str(nGauss)};
                    opts.Interpreter = 'tex';
                    answer2 = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                    nGauss  = str2double(answer2{1});
                else
                    nGauss = varargin{1};
                end

                [R.Xs,R.Ws,R.nip] = GaussQuadrature(nGauss);

            else  % all local (element) basis
                
                n_ele = max(R.Phi_odr); % All numbers must be same max is used considering user error

                if nargin==1
                    nGauss = nGauss_min;
                    
                    if nGauss/n_ele-floor(nGauss/n_ele)==0
                        nGausse = floor(nGauss/n_ele);
                    else
                        nGausse = floor(nGauss/n_ele)+1;
                    end
    
                    switch R.Type
                        case 'FEM Like'
                            if strcmp(R.SubClass,'Linear')
                                nGausse(nGausse<2) = 2; % Minimum nGauss per element
                            elseif strcmp(R.SubClass,'Quadratic')
                                nGausse(nGausse<3) = 3; % Minimum nGauss per element
                            elseif strcmp(R.SubClass,'Cubic')
                                nGausse(nGausse<4) = 4; % Minimum nGauss per element
                            end
                            
                        case {'Gaussian Element', 'Inverse Multi-quadratic'}
                            nGausse(nGausse<3) = 3; % Minimum nGauss per element
                        case 'Hermite Spline'
                            nGausse(nGausse<4) = 4; % Minimum nGauss per element
                    end
                    
                    prompt           = ['Gauss Quadrature points per element (min ', num2str(nGausse),'):'];
                    dlgtitle         = 'Gaussian Points ';
                    definput         = {num2str(nGausse)};
                    opts.Interpreter = 'tex';
                    answer2 = inputdlg(prompt,dlgtitle,[1 70],definput,opts);
                    nGausse = str2double(answer2{1});
                else
                    nGausse = varargin{1};
                end
                
                [Xse,Wse,nipe]=GaussQuadrature(nGausse); %per element

                R.nip = n_ele*(nipe-1)+1;
                R.Ws = zeros(R.nip,1);
                R.Xs = zeros(R.nip,1);
                R.Ws(1:end-1)=repmat(Wse(1:end-1),n_ele,1)/n_ele;
                
                X0 = 0;
                for i=1:n_ele
                    R.Xs((i-1)*(nipe-1)+1:i*(nipe-1)) = X0+Xse(1:nipe-1)/n_ele;
                    X0 = X0+1/n_ele;
                end
                R.Xs(end)=1;
            end
            R.Ws = R.Link.ld{R.div}*R.Ws;
            R.Add_more_X();
        end

        function UpdateWeights(R) %updates nip, Xs, Ws (when ld is changed)
            if isempty(R.dof)
                return
            end

            if any(strcmp(R.Type,{'Monomial','Legendre Polynomial','Chebychev','Fourier','Custom Basis'})) % all global basis
                
                [R.Xs,Ws_new,~] = GaussQuadrature(R.nip-2);
                R.Ws = R.Link.ld{R.div}*Ws_new;

            else  % all local (element) basis
                
                n_ele = max(R.Phi_dof);
                nipe = (R.nip-1)/n_ele+1;

                [Xse,Wse,~]=GaussQuadrature(nipe-2); %per element
                Ws_new = zeros(R.nip,1);
                Xs_new = zeros(R.nip,1);
                
                Ws_new(1:end-1)=repmat(Wse(1:end-1),n_ele,1)/n_ele;
                
                X0 = 0;
                for i=1:n_ele
                    Xs_new((i-1)*(nipe-1)+1:i*(nipe-1)) = X0+Xse(1:nipe-1)/n_ele;
                    X0 = X0+1/n_ele;
                end
                Xs_new(end)=1;
                R.Xs = Xs_new;
                R.Ws = R.Link.ld{R.div}*Ws_new;

            end
            R.Add_more_X();
        end

%% Add more computational points along the rod
        function Add_more_X(R)
            for k=1:length(R.Xadd)
                X1 = R.Xadd(k);
                if ~any(R.Xs==X1) %if already present don't add
                    iv = find(R.Xs>X1);
                    iX1 = iv(1);
                    R.Xs = [R.Xs(1:(iX1-1));X1;R.Xs(iX1:end)];
                    R.Ws = [R.Ws(1:(iX1-1));0;R.Ws(iX1:end)];
                    R.nip = R.nip+1;
                end
            end
        end
%% Update dof of the Rod

        function UpdateDOF(R)

            if isempty(R.dof)
                return
            end
            if any(strcmp(R.Type,{'Monomial','Legendre Polynomial','Chebychev'}))
                R.dof  = sum(R.Phi_dof.*(R.Phi_odr+1));
            elseif strcmp(R.Type,'Fourier')
                R.dof  = sum(R.Phi_dof.*(2*R.Phi_odr+1));
            elseif strcmp(R.Type,'FEM Like')
                n_ele = max(R.Phi_odr);
                n_ele(n_ele==0)=1;
                if isempty(R.SubClass) %Default
                    R.SubClass = 'Linear';
                end
                if strcmp(R.SubClass,'Linear')
                    R.dof = sum(R.Phi_dof*(n_ele+1));
                elseif strcmp(R.SubClass,'Quadratic')
                    R.dof = sum(R.Phi_dof*(2*n_ele+1));
                else % cubic
                    R.dof = sum(R.Phi_dof*(3*n_ele+1));
                end
            elseif strcmp(R.Type,'Hermite Spline')
                n_ele = max(R.Phi_odr);
                n_ele(n_ele==0)=1;
                R.dof  = sum(R.Phi_dof*(2*n_ele+2));  
            elseif any(strcmp(R.Type,{'Gaussian Element','Inverse Multi-quadratic'})) 
                n_ele = max(R.Phi_odr);
                n_ele(n_ele==0)=1;
                R.dof  = sum(R.Phi_dof*(n_ele+1));    
            elseif strcmp(R.Type,'Custom Basis')
                B0     = R.Phi_h(0);
                R.dof  = size(B0,2); 
            end
            
        end
%% Update Precomputed Basis functions

        function UpdatePhis(R)
            if isempty(R.dof)
                return
            end

            Z1     = 1/2-sqrt(3)/6;          %Zanna quadrature coefficient
            Z2     = 1/2+sqrt(3)/6;          %Zanna quadrature coefficient
            Z      = 1/2;                    %Zanna quadrature coefficient 2nd order

            R.Phi    = zeros(R.nip*6,R.dof);
            R.Phi_Z1 = zeros((R.nip-1)*6,R.dof);
            R.Phi_Z2 = zeros((R.nip-1)*6,R.dof);
            R.Phi_Z  = zeros((R.nip-1)*6,R.dof);

            if strcmp(R.Type, 'FEM Like')
                extra_args = {R.Phi_dof, R.Phi_odr, R.SubClass};
            elseif strcmp(R.Type, 'Custom Basis')
                extra_args = {};
            else
                extra_args = {R.Phi_dof, R.Phi_odr};
            end
            
            ld = R.Link.ld{R.div};
            Phi_Scale = diag([1/ld 1/ld 1/ld 1 1 1]);

            X = R.Xs(1);
            R.Phi(1:6, :) = Phi_Scale*R.Phi_h(X, extra_args{:});
            for ii = 2:R.nip
                X = R.Xs(ii);
                R.Phi(6*(ii-1)+1:6*ii,:) = Phi_Scale*R.Phi_h(X,extra_args{:});
                X = R.Xs(ii-1)+Z1*(R.Xs(ii)-R.Xs(ii-1));
                R.Phi_Z1(6*(ii-2)+1:6*(ii-1),:) = Phi_Scale*R.Phi_h(X,extra_args{:});
                X = R.Xs(ii-1)+Z2*(R.Xs(ii)-R.Xs(ii-1));
                R.Phi_Z2(6*(ii-2)+1:6*(ii-1),:) = Phi_Scale*R.Phi_h(X,extra_args{:});
                X = R.Xs(ii-1)+Z*(R.Xs(ii)-R.Xs(ii-1));
                R.Phi_Z(6*(ii-2)+1:6*(ii-1),:) = Phi_Scale*R.Phi_h(X,extra_args{:});
            end

                  
        end
%% Update Precomputed Cross Sectional Inertia, Elastic, and Damping Matrices     
        function UpdateMEG(R)
            if isempty(R.dof)
                return
            end
            [R.Ms,R.Es,R.Gs] = MEG(R.Link,R.div,R.Xs);
        end

        function UpdateEGJoint(R)
            if isempty(R.dof)
                return
            end
            if isequal(size(R.Link.Kj),[R.dof,R.dof])
                R.Es = R.Phi*R.Link.Kj*R.Phi'; %converting to 6x6 matrix
            else
                R.Es = zeros(6,6);
            end

            if isequal(size(R.Link.Dj),[R.dof,R.dof])
                R.Gs = R.Phi*R.Link.Dj*R.Phi'; %converting to 6x6 matrix
            else
                R.Gs = zeros(6,6);
            end
        end
%% Update precomputed reference strains
        function UpdateXi_star(R)

            if isempty(R.dof)
                return
            end

            Z1     = 1/2-sqrt(3)/6;          %Zanna quadrature coefficient
            Z2     = 1/2+sqrt(3)/6;          %Zanna quadrature coefficient
            Z      = 1/2;                    %Zanna quadrature coefficient 2nd order

            R.xi_star        = zeros(6*R.nip,4);
            R.xi_star(1:6,1) = R.xi_starfn(R.Xs(1)); %1
            for ii=2:R.nip
                R.xi_star((ii-1)*6+1:ii*6,1)     = R.xi_starfn(R.Xs(ii)); %2 to nGauss
                R.xi_star((ii-2)*6+1:(ii-1)*6,2) = R.xi_starfn(R.Xs(ii-1)+Z1*(R.Xs(ii)-R.Xs(ii-1))); %1 to nGauss-1
                R.xi_star((ii-2)*6+1:(ii-1)*6,3) = R.xi_starfn(R.Xs(ii-1)+Z2*(R.Xs(ii)-R.Xs(ii-1))); %1 to nGauss-1
                R.xi_star((ii-2)*6+1:(ii-1)*6,4) = R.xi_starfn(R.Xs(ii-1)+Z*(R.Xs(ii)-R.Xs(ii-1))); %1 to nGauss-1
            end
        end

%% Combined Update Functions

        function UpdateAll(R)
            if isempty(R.dof)
                return
            end
            R.UpdateDOF();
            R.UpdateWeights();
            R.UpdatePhis();
            R.UpdateXi_star();
            R.UpdateMEG();
            R.UpdateEGJoint();
        end

        function UpdatePreCompute(R)
            if ~isempty(R.Type) % Condition for closed chain case
                R.UpdatePhis();
                R.UpdateXi_star();
                R.UpdateMEG();
            end
        end

        function Link2RodUpdate(R)
            if ~isempty(R.Type) % Condition for closed chain case
                R.UpdateWeights(); %weights will change if lenght is changed
                R.UpdatePhis(); %scaling factor will change
                R.UpdateMEG(); %rod cross section dimentions or material properties
                R.UpdateEGJoint(); %when Kj or Dj changes
            end
        end


 %% Set functions           
        
        function set.Phi_dof(R, value)
            R.Phi_dof = value;
            R.UpdateDOF();
            R.UpdatePhis();
        end
        function set.Phi_odr(R, value)
            R.Phi_odr = value;
            R.UpdateAll();
        end
        function set.Type(R, value)
            R.Type = value;
            R.UpdatePhi_h();
            R.UpdateAll();
        end
        function set.xi_starfn(R, value)
            R.xi_starfn = value;
            R.UpdateXi_star();
        end
        
        function set.Phi_h(R, value)
            R.Phi_h = value;
            R.UpdateAll();
        end
        function set.SubClass(R, value)
            R.SubClass = value;
            R.UpdateAll();
        end
        
        function set.Xadd(R,value)
            R.Xadd = value;
            R.Add_more_X();
            R.UpdatePreCompute();
        end



        function s = saveobj(Rod) 
            % save properties into a struct to avoid set method when loading
            % Pass that struct onto the SAVE command.
            s.Type = Rod.Type;
            s.SubClass = Rod.SubClass;
            s.Phi_dof = Rod.Phi_dof;
            s.Phi_odr = Rod.Phi_odr;
            s.dof = Rod.dof;
            s.Phi_h = Rod.Phi_h;
            s.Phi = Rod.Phi;
            s.Phi_Z1 = Rod.Phi_Z1;
            s.Phi_Z2 = Rod.Phi_Z2;
            s.Phi_Z = Rod.Phi_Z;
            s.xi_starfn = Rod.xi_starfn;
            s.xi_star = Rod.xi_star;
            s.Link = Rod.Link;
            s.div = Rod.div;
            s.nip = Rod.nip;
            s.Xs = Rod.Xs;
            s.Ws = Rod.Ws;
            s.Ms = Rod.Ms;
            s.Es = Rod.Es;
            s.Gs = Rod.Gs;
            s.Xadd = Rod.Xadd;
        end
  
    end
    
    methods(Static)
        
        function Rod = loadobj(s) 
            Rod = SorosimRod;
            Rod.dof = []; %to overload set methods when loading
            Rod.Type = s.Type;
            Rod.SubClass = s.SubClass;
            Rod.Phi_dof = s.Phi_dof;
            Rod.Phi_odr = s.Phi_odr;
            Rod.Phi_h = s.Phi_h;
            Rod.Phi = s.Phi;
            Rod.Phi_Z1 = s.Phi_Z1;
            Rod.Phi_Z2 = s.Phi_Z2;
            Rod.Phi_Z = s.Phi_Z;
            Rod.xi_starfn = s.xi_starfn;
            Rod.xi_star = s.xi_star;
            Rod.Link = s.Link;
            Rod.div = s.div;
            Rod.nip = s.nip;
            Rod.Xs = s.Xs;
            Rod.Ws = s.Ws;
            Rod.Ms = s.Ms;
            Rod.Es = s.Es;
            Rod.Gs = s.Gs;
            Rod.Xadd = s.Xadd;
            Rod.dof = s.dof;
        end
    
    end
end
