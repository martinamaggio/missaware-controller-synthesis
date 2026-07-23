% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c,mpc] = treempc_param_furuta(strategy_overrun)
    % Parameters used for the Furuta example controller design 
    
    % System dynamics
    [A_c,B_c] = furuta_matrices();
    
    switch strategy_overrun
        case 'kill'
            nx_tilde = size(A_c,1) + 1;
        case 'skip'
            nx_tilde = size(A_c,1) + 2;
        otherwise
            error('Overrun strategy not valid')
    end

    % MPC parameters and constraints
    mpc.N = 5; % prediction horizon
    mpc.Q = 1e-3*eye(nx_tilde);
    mpc.Q(2:end,2:end) = 1e-5*eye(nx_tilde-1);
    mpc.R = 1e-3;

    theta_max = 0.5; % pendulum angle soft constraint 
    tau_max = 0.2; % input hard constraint

    mpc.input_constraint.L = [1; -1];
    mpc.input_constraint.lu = tau_max*ones(size(mpc.input_constraint.L,1),1);
    
    mpc.state_constraint.H = [1, zeros(1,nx_tilde-1); 
                             -1, zeros(1,nx_tilde-1)];
    mpc.state_constraint.hx = theta_max*ones(size(mpc.state_constraint.H,1),1);

    % slack variable penalty function parameter
    mpc.S = 1e2*ones(length(mpc.state_constraint.hx),1);
end