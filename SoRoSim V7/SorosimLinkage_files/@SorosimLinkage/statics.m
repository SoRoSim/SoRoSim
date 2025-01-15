%Function for the static equilibrium function of the linkage
%Last modified by Anup Teejo Mathew 02.03.2022
function [q,u,lambda]=statics(Linkage,qul0,uq,staticsOptions) %qul0_user is initial guess of qul0 arranged like this: [q_u0;u_u0;l0]. uq_user is actuation input like this: [u_k;q_k]

uq = zeros(Linkage.nact,1); %uq has q_k in indexes of of u_u
if Linkage.Actuated
    n_k = Linkage.ActuationPrecompute.n_k; %number of known values of q (for joint controlled systems)
    %Actuation input
    if nargin <= 2 || isempty(uq) 
        if ~Linkage.CAS
            uq_temp(1:Linkage.n_jact) = InputJointUQ0(Linkage); %Rigid Joints. Here q_k is in the indices of u_u
            for i=Linkage.n_jact+1:Linkage.nact %Soft Actuators
                prompt = ['Enter actuation force of the soft actuator ',num2str(i-Linkage.n_jact),' (N):'];
                answer = input(prompt, 's');
                uq_temp(i)  = eval(answer);
            end
            uq = uq_temp;
            uq(1:Linkage.nact-n_k) = uq_temp(Linkage.ActuationPrecompute.index_u_k);
            uq(Linkage.act-n_k+1:Linkage.nact) = uq_temp(Linkage.ActuationPrecompute.index_u_u);
        else
            uq = CustomActuatorStrength(Tr,0);
        end
    end
else
    n_k = 0;
end


%initial guess definition
if nargin == 1 || isempty(qul0)
    
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
    dlgtitle = 'Initial condition';
    opts.Interpreter = 'latex';
    answer = inputdlg(prompt, dlgtitle, [1, 75], definput, opts);

    % Construct qul0 based on input
    qul0 = eval(['[', strjoin(answer, ' '), ']'])';

end

if nargin<=3
    staticsOptions.magnifier = true;
    staticsOptions.Jacobian = true;
end

if staticsOptions.Jacobian
    options = optimoptions('fsolve','Algorithm','trust-region-dogleg','Display','iter','Jacobian','on','MaxFunctionEvaluations',1e7); % Algorithm: 'trust-region-dogleg' (default), 'trust-region', and 'levenberg-marquardt'.
    Func    = @(qul) Equilibrium(Linkage,qul,uq,staticsOptions.magnifier);
else
    options = optimoptions('fsolve','Algorithm','trust-region-dogleg','Display','iter','MaxFunctionEvaluations',1e7); % Algorithm: 'trust-region-dogleg' (default), 'trust-region', and 'levenberg-marquardt'.
    Func    = @(qul) EquilibriumResidue(Linkage,qul,uq,staticsOptions.magnifier); %single pass algorithm only computes Residue
end

disp('Solving Static Equilibrium')

tic
qul = fsolve(Func,qul0,options);
toc

%% save and plot

q = qul(Linkage.ndof,1);
u = uq;
lambda = qul(Linkage.ndof+1:end);


if Linkage.Actuated

    q(Linkage.ActuationPrecompute.index_q_k) = uq(Linkage.ActuationPrecompute.index_u_u);
    u(Linkage.ActuationPrecompute.index_u_u) = qul(Linkage.ActuationPrecompute.index_q_k);

end

varsToSave = {'q'};
if Linkage.Actuated
    varsToSave{end+1} = 'u';
end
if Linkage.nCLj > 0
    varsToSave{end+1} = 'lambda';
end
save('StaticsSolution.mat', varsToSave{:});

Linkage.plotq(q);
end
