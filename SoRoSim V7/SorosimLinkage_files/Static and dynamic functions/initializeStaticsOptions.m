function staticsOptions = initializeStaticsOptions(userOptions)
    % Default values
    defaultOptions.magnifier = true;
    defaultOptions.Jacobian = true;
    defaultOptions.Algorithm = 'trust-region-dogleg';

    % Check if userOptions is provided
    if nargin < 1 || isempty(userOptions)
        staticsOptions = defaultOptions; % Use defaults if no user options are provided
    else
        % Override defaults with user-provided values
        staticsOptions = defaultOptions; % Start with defaults
        fields = fieldnames(userOptions);
        for i = 1:length(fields)
            staticsOptions.(fields{i}) = userOptions.(fields{i});
        end
    end
end
