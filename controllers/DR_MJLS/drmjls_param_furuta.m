% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c,Q,R] = drmjls_param_furuta(strategy_overrun)
    % Parameters used for the Furuta example controller design 
    
    % System dynamics
    [A_c,B_c] = furuta_matrices();
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
    Q = 1e-3*eye(nx_tilde);
    R = 1e-3;
    Q(2:end,2:end) = 1e-5*eye(nx_tilde-1);
end