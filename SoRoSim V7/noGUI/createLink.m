function Link = createLink(filename)
    fid = fopen(filename, 'r'); 
    if fid == -1
        error('Cannot open file for reading.');
    end
    raw = fread(fid, inf, 'uint8=>char')'; % Read JSON file
    fclose(fid);
    
    data = jsondecode(raw); % Convert JSON to struct

    Link = SorosimLink('empty'); %create an empty class element
    
    %To create a rigid link
    Link.jointtype=data.jointtype;
    Link.linktype=data.linktype;
    
    Link.npie = data.npie;
    
    CS = 'C'; %'C' for circular, 'R' for circular, 'E' for circular
    L = 2;
    Link.E = data.E;
    Link.Poi = data.Poi;
    Link.Eta = data.Eta;
    % G = Link.E/(2*(1+Link.Poi));
    Link.G = data.G;
    Rho = 1000;
    % r = @(X1)0.03; %function of normalzied X. X1 in [0 1]
    
    % [M,cx] = RigidBodyProperties(CS,L,Rho,h,w); %for rectangular cross section
    % [M,cx] = RigidBodyProperties(CS,L,Rho,a,b); %for elliptical cross section
    
    Link.ld{1} = data.ld;
    Link.L= data.L;
    Link.CS= data.CS;
    
    Link.h= data.h;
    Link.w= data.h;
    Link.a= data.a;
    Link.b= data.b;
    if Link.linktype == 'r'
        Link.r= str2func(data.r{1});
        Link.gi = data.gi;
        Link.gf = data.gf;
        % [M,cx] = RigidBodyProperties(CS,L,Rho,r); %for circular cross section
        Link.M= data.M;
        Link.cx= data.cx;
    
    else
        Link.r{1}= str2func(data.r{1});
        Link.gi{1} = squeeze(data.gi);
        Link.gf{1} = squeeze(data.gf); %% addmore if there are other divisions
    end
    
    Link.Rho= data.Rho;
    Link.Kj= data.Kj;
    Link.Dj= data.Dj;
    
    Link.n_l= 25;
    Link.n_r= 18; %should be 5 for rectanglular cross section
    Link.color= [0.9572 0.4854 0.8003];
    Link.alpha= 1;
    Link.CPF= false;
    Link.PlotFn= @(g)CustomShapePlot(g);
    Link.Lscale= 0.0947;
end