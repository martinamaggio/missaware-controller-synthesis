% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c,C,design_func] = streamadapt_param_motor(C)
    % Parameters and nominal control design function used for the motor example controller design 

    [A_c,B_c] = motor_matrices();

    function [K,L] = nominal_ctrl_design(A,B,C,~)
        Q_ctrl = diag([30,30,10,0,0]);
        R_ctrl = diag([20,20]);
        Q_obs = 5*Q_ctrl;
        R_obs = R_ctrl;

        K = dlqr(A,B,Q_ctrl,R_ctrl);
        L = dlqr(A',C',Q_obs,R_obs)';
    end

    design_func = @nominal_ctrl_design;

end