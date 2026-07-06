function [sequence] = deadline_anyhit_mandatory(num_iterations, protected, mu, kap, prob_miss)
% Returns a sequence of length num_iterations that admits the weakly-hard
% constraint (AnyMiss(m,kap), where the probability of having a miss (when it
% is possible) is prob_miss. The first protected iterations are always
% hits, i.e., = 1. The sequence additionally satisfies mandatory hit
% sequence of AccControl.

  sequence(1:num_iterations) = ones(1, num_iterations);

  for k = 1:num_iterations
    if floor(ceil(k*mu/kap)*kap/mu) == k
        continue; % mandatory job
    else
        if rand() < prob_miss
            sequence(k) = 0; % optional job missed its deadline
        end
    end
  end

  sequence(1:protected) = ones(1,protected); % protected iterations are hits
  
end
