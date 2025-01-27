function dynamicsOptions = initializeDynamicsOptions(userOptions)
    % Default values

    defaultOptions.dt = 0.01;
    defaultOptions.Jacobian = true;
    defaultOptions.displayTime = true; %display time
    defaultOptions.plotProgress = false; %to plot robot configuration as simulation progresses
    defaultOptions.Integrator = 'ode15s';
    defaultOptions.t_start = 0;
    defaultOptions.t_end = 0;

    % Check if userOptions is provided
    if nargin < 1 || isempty(userOptions)
        dynamicsOptions = defaultOptions; % Use defaults if no user options are provided
    else
        % Override defaults with user-provided values
        dynamicsOptions = defaultOptions; % Start with defaults
        fields = fieldnames(userOptions);
        for i = 1:length(fields)
            dynamicsOptions.(fields{i}) = userOptions.(fields{i});
        end
    end
    
end
