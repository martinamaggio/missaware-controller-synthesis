% TEMPLATE_LINEAR solves a linear state-space system using ode45 (template)
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [t, x, y] = template_linear(x0, final_time, u)
% Solves a linear state-space system using ode45.
%
% Inputs
%   x0          : initial state vector
%   final_time  : end time of the integration interval
%   u           : constant system input
%
% Outputs
%   t           : time vector of 20 evenly spaced time steps between 0 and final_time
%   x           : state trajectory matrix
%   y           : output trajectory matrix

  [A,B] = template_matrices();

  fx = @(t,x)[A*x + B*u];
    
  time_interval = [0 final_time];
  solution = ode45(fx, time_interval, x0);
    
  % generate result vectors
  t = linspace(0, final_time, 20)';
  x = deval(solution, t)';
  y = x;

end

