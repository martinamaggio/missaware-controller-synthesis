% FURUTA_ENFORCEDHITS defines when deadline hits are enforced
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function flag = furuta_enforcedhits(x)
% Defines when deadline hits are enforced (e.g., for an initialization procedure).
%
% Inputs
%   x    : system state
%
% Outputs
%   flag : true if deadline hits are enforced

    flag = abs(x(1))>0.5; % swing-up controller active if |angle| > 0.5 rad
end

