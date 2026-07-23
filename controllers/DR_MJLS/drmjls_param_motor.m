% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c,Q,R] = drmjls_param_motor(strategy_overrun)
    % Parameters used for the electric motor example controller design 
    
    % System dynamics
    [A_c,B_c] = motor_matrices();
    nu = size(B_c,2);
    
    % cost matrices
    switch strategy_overrun
        case 'kill'
            nx_tilde = size(A_c,1) + 1*nu;
        case 'skip'
            nx_tilde = size(A_c,1) + 2*nu;
        otherwise
            error('Overrun strategy not valid')
    end
    Q = eye(nx_tilde);
    R = eye(nu);
end