%Function for the dynamic simulation of the linkage
%Last modified by Anup Teejo Mathew 23/05/2021
function [t,qqd] = dynamics(Linkage,x0,dynamicAction,userOptions) %x0 = [q0;qd0], dynamicAction(t) = [0 0 u_k;q_k,qd_k,qdd_k]

ndof  = Linkage.ndof;
GUI_actionInput = false;

if nargin<=3||isempty(userOptions)
    userOptions = [];
end
dynamicsOptions = initializeDynamicsOptions(userOptions);
t_start = dynamicsOptions.t_start;

%Actuation input
if nargin <= 2 || isempty(dynamicAction)
    if Linkage.Actuated
        GUI_actionInput = true; %in this case dynamicAction is a cell of nact elements. For soft and wrench controlled joints each element is a function of time
                          %for joint coordinate controlled joints each element is a cell of 3 elements denoting q_k, qd_k and qdd_k
        dynamicAction_temp = cell(Linkage.nact,1); %dynamicAction_temp has q_k and its derivatives in the indices of u_u
        n_k = Linkage.ActuationPrecompute.n_k; %number of known values of q (for joint controlled systems)

        if ~Linkage.CAI
            dynamicAction_temp(1:Linkage.n_jact) = InputJointUQt(Linkage); %Rigid Joints. Here q_k is in the indices of u_u
            for ia=Linkage.n_jact+1:Linkage.nact %Soft Actuators
                prompt = ['Enter the strength (N) of the soft actuator ',num2str(ia-Linkage.n_jact),' as a function of t (time)'...
                          '\n[Examples: -10-5*t, -50*t+(50*t-50)*heaviside(t-1), 50*sin(2*pi*t)]: '];
                funstr = input(prompt, 's');
                dynamicAction_temp{ia} = str2func( ['@(t) ' funstr ] );
            end
            dynamicAction = dynamicAction_temp; % to convert to u_k, q_k form
            dynamicAction(1:Linkage.nact-n_k) = dynamicAction_temp(Linkage.ActuationPrecompute.index_u_k);
            dynamicAction(Linkage.nact-n_k+1:Linkage.nact) = dynamicAction_temp(Linkage.ActuationPrecompute.index_u_u);
            
        else
            dynamicAction = [];
        end
    else
        dynamicAction = [];
        n_k = 0;
    end
else
    
    if Linkage.Actuated
        n_k = Linkage.ActuationPrecompute.n_k;
        [action,q_k,qd_k] = dynamicAction(t_start);
        if ~(length(q_k)==n_k&&length(qd_k)==n_k&&length(action)==Linkage.nact)
            error('Output dimension mismatch for dynamicActionInput(t_start)');
        end
    else
        dynamicAction = [];
        n_k = 0;
    end
end


%initial guess definition: reference configuration
if nargin==1||isempty(x0)%if initial condition is not given
    q0                 = zeros(1,ndof);
    ndof               = Linkage.ndof;
    qd0                = zeros(1,ndof);
    
    if Linkage.Actuated&&n_k>0
        q0(Linkage.ActuationPrecompute.index_q_k)  = [];
        qd0(Linkage.ActuationPrecompute.index_q_k) = [];
        prompt = {'$\mathbf{q}_{u0}$','$\dot{\mathbf{q}}_{u0}$'};
    else
        prompt = {'$\mathbf{q}_{0}$','$\dot{\mathbf{q}}_{0}$'};
    end
    
    dlgtitle         = 'Initial condition';
    definput         = {num2str(q0),num2str(qd0)};
    opts.Interpreter = 'latex';
    answer           = inputdlg(prompt,dlgtitle,[1 75],definput,opts);

    q0   = eval(['[',answer{1},']'])';
    qd0  = eval(['[',answer{2},']'])';

else
    n0 = Linkage.ndof;
    if Linkage.Actuated
        n0=n0-Linkage.ActuationPrecompute.n_k;
    end
    if ~(length(x0)==2*n0)
        error('Invalid dimension for initial condition: expected %d, but got %d.', 2*n0, length(x0));
    end
    q0  = x0(1:n0); %q_u
    qd0 = x0(n0+1:2*n0); %qd_u
end

if Linkage.Actuated
    q0_pass          = zeros(ndof,1);
    qd0_pass         = zeros(ndof,1);

    q0_pass(Linkage.ActuationPrecompute.index_q_u)  = q0;
    qd0_pass(Linkage.ActuationPrecompute.index_q_u) = qd0;

    q0  = q0_pass;
    qd0 = qd0_pass;

    for ia=Linkage.nact-n_k+1:Linkage.nact
        if GUI_actionInput
            q0(Linkage.ActuationPrecompute.index_q_k(ia+n_k-Linkage.nact))  = dynamicAction{ia}{1}(t_start);
            qd0(Linkage.ActuationPrecompute.index_q_k(ia+n_k-Linkage.nact)) = dynamicAction{ia}{2}(t_start);
        else
            [~,q_k0,qd_k0] = dynamicAction(0);
            q0(Linkage.ActuationPrecompute.index_q_k(ia+n_k-Linkage.nact))  = q_k0(ia-Linkage.nact+n_k);
            qd0(Linkage.ActuationPrecompute.index_q_k(ia+n_k-Linkage.nact)) = qd_k0(ia-Linkage.nact+n_k);
        end
    end
end
x0 = [q0;qd0];

if dynamicsOptions.t_end==0
    prompt           = 'Simulation time (s)';
    dlgtitle         = 'Initial condition';
    definput         = {'5'};
    opts.Interpreter = 'tex';
    answer           = inputdlg(prompt,dlgtitle,[1 50],definput,opts);
    
    tmax = eval(answer{1});
else
    tmax = dynamicsOptions.t_end;
end


dt = dynamicsOptions.dt;
if dynamicsOptions.Jacobian
    options = odeset('OutputFcn',@(t,y,flag) odeprogress(t,y,flag,Linkage,dynamicsOptions.plotProgress),'RelTol',1e-3,'AbsTol',1e-6,'Jacobian',@(t,x) ODEJacobian(Linkage,t,x,dynamicAction,GUI_actionInput));% CHANGE NAMES
else
    options = odeset('OutputFcn',@(t,y,flag) odeprogress(t,y,flag,Linkage,dynamicsOptions.plotProgress),'RelTol',1e-3,'AbsTol',1e-6);% default values'RelTol',1e-3,'AbsTol',1e-6
end

ODEFn = @(t,x) Linkage.derivatives(t,x,dynamicAction,GUI_actionInput,dynamicsOptions); %ODE function which computes [qd;qdd]

switch dynamicsOptions.Integrator
    case 'ode45'
        tic
        [t,qqd] = ode45(ODEFn,t_start:dt:tmax,x0,options);
        toc
    case 'ode23'
        tic
        [t,qqd] = ode23(ODEFn,t_start:dt:tmax,x0,options);
        toc
    case 'ode113'
        tic
        [t,qqd] = ode113(ODEFn,t_start:dt:tmax,x0,options);
        toc
    case 'ode23s'  
        tic
        [t,qqd] = ode23s(ODEFn,t_start:dt:tmax,x0,options);
        toc
    case 'ode15s'
        tic
        [t,qqd] = ode15s(ODEFn,t_start:dt:tmax,x0,options);
        toc
    case 'ode1'
        tic
        qqd = ode1(ODEFn,t_start:dt:tmax,x0);
        toc
        t=0:dt:tmax;
    case 'ode2'
        tic
        qqd = ode2(ODEFn,t_start:dt:tmax,x0);
        toc
        t=0:dt:tmax;
end

save('DynamicsSolution.mat','t','qqd');

answer = questdlg('Generate output video of the simulation?','Grapical Output','Yes','No','Yes');
if strcmp('Yes',answer)
    plotqt(Linkage,t,qqd);
end


end

