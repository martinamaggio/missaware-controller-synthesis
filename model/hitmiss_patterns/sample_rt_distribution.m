% SAMPLE_RT_DISTRIBUTION defines a hit/miss sequence based on a control
% task response time distribution
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [sequence,row_m_max] = sample_rt_distribution(num_iterations, strategy, period, rt_dist, m)
% Returns an outcome sequence built by translating sampled job response times 
% into deadline hits/misses depending on the applied overrun strategy.
%
% Inputs
%   num_iterations  : length of sequence of outcomes
%   strategy        : overrun strategy ('kill' or 'skip')
%   period          : sample period
%   rt_dist         : response time distribution (e.g., @(n) unifrnd(0, 2*period, n, 1))
%   m               : if not empty: rowMiss(m) constraint is imposed on sequence
%
% Outputs
%   sequence        : sequence of ones (hits) and zeros (misses)
%   row_m_max       : minimum value such that the sequence follows the rowMiss(row_m_max) constraint


  sequence(1:num_iterations) = ones(1, num_iterations);

  row_m = 0;
  row_m_max = 0;
  job_times = rt_dist(num_iterations);
  for i = 1:num_iterations
      switch strategy
          case 'kill'
              if job_times(1) > period
                  sequence(i) = 0;
              end
              job_times(1) = [];
          case 'skip'
              if job_times(1) > period
                  job_times(1) = job_times(1) - period;
                  if length(job_times)>1
                    job_times(2) = []; % skip the next job if there is an ongoing job
                  end
                  sequence(i) = 0;
              else
                  job_times(1) = [];
              end
          otherwise
              error('Only overrun strategies kill and skip-next are implemented')
      end
      % determine the number of consecutive misses at the moment
      if sequence(i) == 0
          row_m = row_m+1;
      else
          row_m = 0;
      end
      % enforce a rowMiss(m) constraint if given
      if ~isempty(m) && row_m > m
          sequence(i) = 1;
          row_m = m;
      end
      % determine the maximum number of consecutive misses
      if row_m > row_m_max
          row_m_max = row_m;
      end
  end
  
end