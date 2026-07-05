function [u, z, t_ctrl] = furuta_base(linear_controller, x, z, u, sequence, k)

  % linear controller is a pointer to the function that is going to be
  % called during the linear control phase
  % assumption: all the controllers have the same signature
  %    parameters
  %      - x: system state
  %      - z: controller state
  %      - u: old control signal
  %      - sequence: vector with 1s and 0s with deadline pattern history
  %      - k: time step

  umax = 0.2; umin = -0.2; % saturations
  a = x(1); v = x(2); % upacking angle and angular velocity for swingup
  t_ctrl = []; % online computation time of linear controller

  if abs(a) > 3.0 && abs(v) < 0.5 % initial kick
    u = 0.05; 
  
  elseif abs(a) > 0.5 % swingup procedure
    c = cos(a);
    u = 0.000075 * (c^4) * v * (9.81 * (1-c) - 0.0075 * (v^2));

  else % linear controller (here we are comparing alternatives)
    t_ctrl_start = tic;
    [u,z] = linear_controller(x, z, u, sequence, k);
    t_ctrl = toc(t_ctrl_start);
  end

  u = max(u, umin); u = min(u, umax); % saturations
  if isempty(z)
      z = u;
  end

end
