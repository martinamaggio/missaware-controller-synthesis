% SETUP_TEMPLATE template for adding a new controller synthesis method to the framework
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [ctrl_func, z0] = setup_template()
% Template for adding a new controller synthesis method to the framework.
%
% Outputs
%   ctrl_func           : function handle for the controller
%   z0                  : initial controller state (stateless: z0=[])

    function [u,z] = ctrl_wrapper(x,z,u,sequence,k)
    % Control wrapper to use in simulation script
    end

    ctrl_func = @ctrl_wrapper;
    z0 = [];
end