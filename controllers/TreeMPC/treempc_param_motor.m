% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c,mpc] = treempc_param_motor(strategy_overrun)
    % Parameters used for the electric motor example controller design 
    
    % System dynamics
    [A_c,B_c] = motor_matrices();
    nu = size(B_c,2);
    
    switch strategy_overrun
        case 'kill'
            nx_tilde = size(A_c,1) + 1*nu;
        case 'skip'
            nx_tilde = size(A_c,1) + 2*nu;
        otherwise
            error('Overrun strategy not valid')
    end

    % MPC parameters and constraints
    mpc.N = 3; % prediction horizon
    mpc.Q = 1e-3*eye(nx_tilde);
    mpc.R = 1e-3*eye(nu);

    u_max = 12; % voltage limits
    i_max = 10; % i_q soft constraint

    mpc.input_constraint.L = [ 1,  0;
                              -1,  0;
                               0,  1;
                               0, -1];
    mpc.input_constraint.lu = u_max*ones(size(mpc.input_constraint.L,1),1);
    
    mpc.state_constraint.H = [1, zeros(1,nx_tilde-1); 
                             -1, zeros(1,nx_tilde-1)];
    mpc.state_constraint.hx = i_max*ones(size(mpc.state_constraint.H,1),1);

    % slack variable penalty function parameter
    mpc.S = 1e1*ones(length(mpc.state_constraint.hx),1);

end