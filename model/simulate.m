function [t, x, y, z, u, t_ctrl] = simulate(psys, prt, ctl, sequence, noise_sequence)
% SIMULATE
%
% Inputs
%   psys          : system parameter struct
%   prt           : real-time parameters struct
%   ctl           : controller to be used for closing the loop
%   sequence      : the sequence of hits and misses
%   noise_sequence: sequence of noise to be added to the system state
%
% Outputs
%   t, x, y, z, u : simulation results: time t, state x, output y,
%                   controller state z, inputs u
% t_ctrl          : online computation time of the base controller

plant_name = psys.plant_name;
x0 = psys.x0;
u0 = psys.u0;
z0 = psys.z0;
ctl_period = psys.period;
deadline_strategy = prt.overrun_strategy;
actuation_strategy = prt.actuator_strategy;

plant_sim = str2func([plant_name,'_',psys.plant_sim]);
plant_ctrl_base = str2func([plant_name,'_base']);
enforced_hits = str2func([plant_name,'_enforcedhits']);

  % initialization of results and intermediate values
  t = []; x = []; y = []; z = []; u = []; t0 = 0; t_ctrl = [];
  digital_state_length = 20;
  % pad the sequence with hits before the start of the simulation
  sequence = padarray(sequence,[0,digital_state_length-1],1,'pre'); 

  % calculating delta matrix for actuation strategy
  % in case of hold: identity keeps the control signal
  % in case of zero: zero matrix zeroes the control signal
  delta = eye(size(u0, 1));
  if actuation_strategy == "zero"
    delta = 0 * delta;
  end

  x0known = x0;
  for p = digital_state_length : length(sequence)
    
    u1 = delta * u0; z1 = z0; % basic in case of deadline miss
    t_ctrl_linear = [];
    if enforced_hits(x0known) || sequence(p) == 1 % if I have a deadline hit calculate next control signal
      current_sequence = sequence(p-(digital_state_length-1):p);
      [u1, z1, t_ctrl_linear] = plant_ctrl_base(ctl, x0known, z0, u0, current_sequence, p-(digital_state_length-1));
    end

    [tv, xv, yv] = plant_sim(x0, ctl_period, u0);
    xv(end,:) = xv(end,:) + noise_sequence(:,p-(digital_state_length-1))';

    % saving results for the final plotting
    t = [t; tv+t0]; x = [x; xv]; y = [y; yv]; u = [u; repmat(u0',length(tv),1)];
    t_ctrl = [t_ctrl; t_ctrl_linear];

    % preparing the next iteration
    % in case I skip the deadline with a miss I am not going to change the
    % measurements and the controller with recalculate the same value when
    % there is a hit -- also no change in controller state if any
    if ~enforced_hits(x0known) && is_first_skip_overrun(deadline_strategy,sequence,p)
      x0known = x0;
      z0 = z0; % no change in z0 -- useless statement, but good to see
    elseif ~enforced_hits(x0known) && is_job_ongoing(deadline_strategy,sequence,p)
      x0known = x0known;
      z0 = z0;
    else
      x0known = xv(end, :)'; % I finished so next time I'll measure
      z0 = z1;
    end
    x0 = xv(end, :)'; % measuring new, in case I am not skipping
    u0 = u1; t0 = t0 + ctl_period;

  end
    
end


function flag = is_first_skip_overrun(deadline_strategy,sequence,p)
    flag = deadline_strategy == "skip" && sequence(p-1) == 1 && sequence(p) == 0;
end

function flag = is_job_ongoing(deadline_strategy,sequence,p)
    flag = deadline_strategy == "skip" && sequence(p-1) == 0 && sequence(p) == 0;
end