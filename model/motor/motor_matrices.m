% MOTOR_MATRICES: gives the plant's linear state-space description
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A,B,p] = motor_matrices()
% Gives the plant's linear state-space description
%
% Outputs
%   A   : continuous-time system matrix
%   B   : continuous-time input matrix
%   p   : parameter struct (for usage in nonlinear system model)

  % constants
  p.R = 0.01;         % winding resistance [Ohm]
  p.L_d = 1e-4;       % inductance in d direction [H]
  p.L_q = 1.2e-4;     % inductance in q direction [H]
  p.w_el = 300;       % electrical angular velocity in the operation point [rad/s]
  p.psi_pm = 0.05;    % pm flux linkage [Wb]
  p.p = 6;            % pole pairs
  p.J_m = 0.005;      % inertia [kg m^2]
  p.M_L = 0;          % load torque [Nm]

  % the state x is
  % [ current in d direction,
  %   current in q direction,
  %   electrical angular velocity]

  % the input u is
  % [ voltage in d direction,
  %   voltage in q direction ]

  % definition of the linear model including w_el
  A = [-p.R/p.L_d,          p.L_q*p.w_el/p.L_d,         0; ...
       -p.L_d*p.w_el/p.L_q, -p.R/p.L_q,                 -p.psi_pm/p.L_q; ...
       0,                   3/2*p.p^2/p.J_m*p.psi_pm,   0];
  B = [1/p.L_d, 0;
       0,       1/p.L_q;
       0,       0];


end