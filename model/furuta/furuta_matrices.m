function [A,B,alpha,beta,gamma,delta] = furuta_matrices()
% FURUTA_MATRICES gives the plant's linear state-space description
%
% Outputs
%   A                       : continuous-time system matrix
%   B                       : continuous-time input matrix
%   alpha,betta,gamma,delta : parameters (for usage in nonlinear system model)

  % constants of the pendulum
  l = 0.075;         % length of the pendulum arm [m]
  r = 0.043;         % base radius [m]
  g = 9.81;          % gravity [m/s^2]
  m = 0.0054;        % pendulum mass [kg]
  J = 125 * 1e-6;    % moment of inertia [kg m^2]

  alpha = J + m*r^2;
  beta = 1/3*m*l^2;
  gamma = 1/2*m*r*l;
  delta = 1/2*l*g*m;

  % the state x is
  % [ pendulum angle,
  %   pendulum angular velocity
  %   base angular velocity ]

  % definition of the linear model
  A = [0 1 0; ...
     alpha*delta/(alpha*beta-gamma^2) 0 0; ...
     -delta*gamma/(alpha*beta - gamma^2) 0 0];
  B = [0 ; -gamma/(alpha*beta - gamma^2); beta/(alpha*beta - gamma^2)];

end