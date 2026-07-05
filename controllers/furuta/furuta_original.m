function [u, z] = furuta_original(x, z, u, sequence, ~)

  pa = x(1); % pendulum angle
  pv = x(2); % pendulum velocity
  bv = x(3); % base velocity

  u = 0.3750 * pa + 0.0250 * pv + 0.0125 * bv;
end
