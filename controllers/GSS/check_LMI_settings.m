function lmiSettings = check_LMI_settings(lmiSettings)
%CHECK LMI SETTINGS
% This function checks the specified LMI settings for consistency and
% recovers default values if necessary.
%
% INPUT & OUTPUT
%   lmiSettings: A struct containing the LMI settings with the fields
%           .bUseSlackVariable:  true/false, whether to use the slack
%                                variable G in the LMIs (default: true)
%           .bVariableSlack:     true/false, whether to use a variable
%                                slack (default: true)
%           .tol:                numerical value > 0, the tolerance for LMI
%                                satisfaction (default: 1e-2)
%           .bSilent:            true/false, whether to suppress solver
%                                output (default: true)
%           .bSwitched:          copies to (overwrites) bVariableSlack


% Check setting consistency and recover defaults if necessary
if isfield(lmiSettings,'bUseSlackVariable')
    if ~islogical(lmiSettings.bUseSlackVariable)
        error('Specified LMI tolerance is not a valid numerical value.')
    end
else
    lmiSettings.bUseSlackVariable = true;
end
if isfield(lmiSettings,'bSwitched') %copy setting to bVariableSlack
    lmiSettings.bVariableSlack = lmiSettings.bSwitched;
end
if isfield(lmiSettings,'bVariableSlack')
    if ~islogical(lmiSettings.bVariableSlack)
        error('Invalid slack variable setting.')
    end
else
    lmiSettings.bVariableSlack = true;
end
if isfield(lmiSettings,'tol')
    if ~isfloat(lmiSettings.tol) || lmiSettings.tol <= 0
        error('Specified LMI tolerance is not a valid numerical value.')
    end
else
    lmiSettings.tol = 1e-2;
end
if isfield(lmiSettings,'bSilent')
    if ~islogical(lmiSettings.bSilent)
        error('Invalid silent setting.')
    end
else
    lmiSettings.bSilent = true;
end


end