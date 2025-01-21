%Function for the static equilibrium function of the linkage
%Last modified by Anup Teejo Mathew 02.03.2022
function [q,u,lambda]=statics(Linkage,x0,input,userOptions) %x0 is initial guess of unknowns arranged like this: [q_u0;u_u0;l0]. input is actuation input like this: [u_k;q_k]

%Actuation input
if nargin <= 2 || isempty(input)
    input_temp = zeros(Linkage.nact,1); %input_temp has q_k in the indices of u_u
    if Linkage.Actuated
        n_k = Linkage.ActuationPrecompute.n_k; %number of known values of q (for joint controlled systems)
        %Actuation input
        if nargin <= 2 || isempty(input) 
            if ~Linkage.CAI
                input_temp(1:Linkage.n_jact) = InputJointUQ0(Linkage); %Rigid Joints. Here q_k is in the indices of u_u
                for i=Linkage.n_jact+1:Linkage.nact %Soft Actuators
                    prompt = ['Enter actuation force of the soft actuator ',num2str(i-Linkage.n_jact),' (N):'];
                    answer = input(prompt, 's');
                    input_temp(i)  = eval(answer);
                end
                input = input_temp;
                input(1:Linkage.nact-n_k) = input_temp(Linkage.ActuationPrecompute.index_u_k);
                input(Linkage.nact-n_k+1:Linkage.nact) = input_temp([Linkage.ActuationPrecompute.index_u_u,Linkage.n_jact+1:Linkage.nact]);
            else
                input = zeros(Linkage.nact,1);
            end
        end
    else
        input = [];
        n_k = 0;
    end
else
    if Linkage.Actuated
        n_k = Linkage.ActuationPrecompute.n_k;
    else
        input = [];
        n_k = 0;
    end
end


%initial guess definition
if nargin == 1 || isempty(x0)
    
    q_u0 = zeros(1,Linkage.ndof - n_k);
    u_u0 = zeros(1,n_k);

    if any(~Linkage.WrenchControlled) && Linkage.nCLj > 0
        lambda_0 = zeros(1,Linkage.CLprecompute.nCLp);
        prompt = {'$\mathbf{q}_{u0}$ (unknown)', '$\mathbf{u}_{u0}$ (unknown)', '$\mathbf{\lambda}_{0}$'};
        definput = {num2str(q_u0), num2str(u_u0), num2str(lambda_0)};
    elseif any(~Linkage.WrenchControlled)
        prompt = {'$\mathbf{q}_{u0}$ (unknown)', '$\mathbf{u}_{u0}$ (unknown)'};
        definput = {num2str(q_u0), num2str(u_u0)};
    elseif Linkage.nCLj > 0
        lambda_0 = zeros(1,Linkage.CLprecompute.nCLp);
        q_0 = zeros(1,Linkage.ndof);
        prompt = {'$\mathbf{q}_{0}$', '$\mathbf{\lambda}_{0}$'};
        definput = {num2str(q_0), num2str(lambda_0)};
    else
        q_0 = zeros(1,Linkage.ndof);
        prompt = {'$\mathbf{q}_{0}$'};
        definput = {num2str(q_0)};
    end

    % Common input dialog handling
    dlgtitle = 'Initial Guess';
    opts.Interpreter = 'latex';
    answer = inputdlg(prompt, dlgtitle, [1, 75], definput, opts);

    % Construct x0 based on user input of initial guess
    x0 = eval(['[', strjoin(answer, ' '), ']'])';

end

if nargin<=3||isempty(userOptions)
    userOptions = [];
end
staticsOptions = initializeStaticsOptions(userOptions);

if staticsOptions.Jacobian
    options = optimoptions('fsolve','Algorithm',staticsOptions.Algorithm,'Display','iter','Jacobian','on','MaxFunctionEvaluations',1e7);
    Func    = @(x) Equilibrium(Linkage,x,input,staticsOptions.magnifier); %two pass RNEA algorithm
else
    options = optimoptions('fsolve','Algorithm',staticsOptions.Algorithm,'Display','iter','MaxFunctionEvaluations',1e7); 
    Func    = @(x) EquilibriumResidue(Linkage,x,input,staticsOptions.magnifier); %write single pass algorithm only computes Residue
end

disp('Solving Static Equilibrium')

tic
x = fsolve(Func,x0,options);
toc

%% save and plot

q = x(1:Linkage.ndof,1);
u = input;
lambda = x(Linkage.ndof+1:end);


if Linkage.Actuated

    q(Linkage.ActuationPrecompute.index_q_u) = x(1:Linkage.ndof-n_k);
    q(Linkage.ActuationPrecompute.index_q_k) = input(end-n_k+1:end);
    u(Linkage.ActuationPrecompute.index_u_k) = input(1:Linkage.nact-n_k);
    u(Linkage.ActuationPrecompute.index_u_u) = x(Linkage.ndof-n_k+1:Linkage.ndof);

end

varsToSave = {'q','x'};
if Linkage.Actuated
    varsToSave{end+1} = 'u';
end
if Linkage.nCLj > 0
    varsToSave{end+1} = 'lambda';
end
save('StaticsSolution.mat', varsToSave{:});

Linkage.plotq(q);
end
