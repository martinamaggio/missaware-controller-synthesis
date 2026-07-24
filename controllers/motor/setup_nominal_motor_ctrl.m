% SETUP_NOMINAL_MOTOR_CTRL defines a dynamic controller with no deadline-miss awareness for an electric motor
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [ctrl, z0] = setup_nominal_motor_ctrl(sampling_period,C,baseline_ctrl)
    % Defines a dynamic controller with no deadline-miss
    % awareness for the linear motor plant and returns the control wrapper
    %
    % Inputs
    %   sampling_period : control period
    %   C               : output matrix
    %   baseline_ctrl   : control design function
    %
    % Outputs
    %   ctrl            : function handle for the controller
    %   z0              : initial controller state

    % baseline controller design
    [F,G,H,K,~] = baseline_ctrl(sampling_period,C);

    function [u,z] = ctrl_nominal(x,z,~,~,~)
        % Control wrapper to use in simulation script
        % Inputs
        %   x   : system state
        %   z   : controller state

        y = C*x; % actual output measurement

        u = H*z + K*y;
        z = F*z + G*y;

    end

    ctrl = @ctrl_nominal;
    z0 = zeros(size(F,2),1);
end