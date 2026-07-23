% FURUTA_AT_OP defines when the plant is close to its operating point
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function flag = furuta_at_op(x)
% Defines when the plant is close to its operating point.
%
% Inputs
%   x    : system state
%
% Outputs
%   flag : true if plant is close to its operating point

    tol_angle = 0.2;
    if abs(x(1))<tol_angle
        flag = 1; 
    else
        flag = 0;
    end
end

