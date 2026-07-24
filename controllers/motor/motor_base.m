% MOTOR_BASE calls the appropriate control function and applies saturation 
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [u, z, t_ctrl] = motor_base(linear_controller, x, z, u, sequence, k)
% Control base function that calls the linear controller, applies saturation 
% to the input, and measures online computation times.
%
% Inputs
%   linear_controller   : function handle of linear controller
%   x, z, u             : system state, controller state, and last input
%   sequence            : window of hit/miss sequence up to time k 
%   k                   : current time step
%
% Outputs
%   u, z                : new input and controller state
%   t_ctrl              : online computation time

  umax = 12; umin = -12; % saturations
  t_ctrl = []; % online computation time of linear controller

  t_ctrl_start = tic;
  [u,z] = linear_controller(x, z, u, sequence, k);
  t_ctrl = toc(t_ctrl_start);

  u = max(u, umin); u = min(u, umax); % saturations
  if isempty(z)
      z = u;
  end

end
