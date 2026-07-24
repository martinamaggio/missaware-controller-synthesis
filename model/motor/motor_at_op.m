% MOTOR_AT_OP defines when the plant is close to its operating point
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function flag = motor_at_op(x)
% Defines when the plant is close to its operating point
%
% Inputs
%   x    : system state
%
% Outputs
%   flag : true if plant is close to its operating point

    tol_id = 0.05;
    tol_iq = 0.05;
    tol_wel = 0.05;
    if abs(x(1))<tol_id && abs(x(2))<tol_iq && abs(x(3))<tol_wel 
        flag = 1; 
    else
        flag = 0;
    end
end

