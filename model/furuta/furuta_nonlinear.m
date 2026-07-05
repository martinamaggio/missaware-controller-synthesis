function [t, x, y] = furuta_nonlinear(x0, final_time, u)
% FURUTA_NONLINEAR solves a nonlinear state-space system using ode45
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

  [~,~,alpha,beta,gamma,delta] = furuta_matrices();

  % the state x is
  % [ pendulum angle,
  %   pendulum angular velocity
  %   base angular velocity ]

  % definition of the nonlinear model
  fx = @(t, x) [ ...
      x(2);
      1/(alpha*beta-gamma^2+(beta^2+gamma^2)*sin(x(1))^2) * ...
        (beta*(alpha+beta*sin(x(1))^2)*cos(x(1))*sin(x(1))*x(3)^2+ ...
        2*beta*gamma*(1-(sin(x(1))^2))*sin(x(1))*x(3)*x(2)- ...
        gamma^2*cos(x(1))*sin(x(1))*x(2)^2+ ...
        delta*(alpha+beta*(sin(x(1)))^2)*sin(x(1))- ...
        gamma*cos(x(1))*u);
      1/(alpha*beta-gamma^2+(beta^2+gamma^2)*sin(x(1))^2) * ...
        (beta*gamma*(sin(x(1))^2-1)*sin(x(1))*x(3)^2- ...
        2*beta^2*cos(x(1))*sin(x(1))*x(3)*x(2)+ ...
        beta*gamma*sin(x(1))*x(2)^2- ...
        gamma*delta*cos(x(1))*sin(x(1))+ ...
        beta*u) ...
    ]; 

  time_interval = [0 final_time];
  solution = ode45(fx, time_interval, x0);
    
  % generate result vectors
  t = linspace(0, final_time, 20)';
  x = deval(solution, t)';
  x(:,1) = wrapToPi(x(:,1)); % angle is between -pi and pi
  y = x;

end