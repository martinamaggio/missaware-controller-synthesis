% MOTOR_NONLINEAR solves a nonlinear state-space system using ode45
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [t, x, y] = motor_nonlinear(x0, final_time, u)
% Solves a nonlinear state-space system using ode45
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

  [~,~,p] = motor_matrices();

  % the state x is
  % [ current in d direction,
  %   current in q direction,
  %   electrical angular velocity ]

  % the input u is
  % [ voltage in d direction,
  %   voltage in q direction ]

  % definition of the nonlinear model
  fx = @(t, x) [ ...
        -p.R/p.L_d * x(1) + p.L_q/p.L_d * x(2)*x(3) + 1/p.L_d * u(1);
        -p.L_d/p.L_q * x(1)*x(3) - p.R/p.L_q * x(2) + 1/p.L_q * u(2) - p.psi_pm/p.L_q * x(3);
        3/2*p.p^2/p.J_m * ((p.L_d-p.L_q) * x(1)*x(2) + p.psi_pm * x(2)) - p.p/p.J_m*p.M_L ...
        ];

  time_interval = [0 final_time];
  solution = ode45(fx, time_interval, x0);
    
  % generate result vectors
  t = linspace(0, final_time, 20)';
  x = deval(solution, t)';
  y = x;

end