function [sequence] = deadline_anymiss(num_iterations, protected, m, k, prob_miss)
% Returns a sequence of length num_iterations that admits the weakly-hard
% constraint (AnyMiss(m,k), where the probability of having a miss (when it
% is possible) is prob_miss. The first protected iterations are always
% hits, i.e., = 1.

  sequence(1:num_iterations) = ones(1, num_iterations);

  for i = 1:num_iterations
    window = sequence(max(i-k+1, 1):i-1);
    misses = sum(window == 0);
    if misses < m && rand() < prob_miss
      sequence(i) = 0;
    end
  end

  sequence(1:protected) = ones(1, protected); 

end