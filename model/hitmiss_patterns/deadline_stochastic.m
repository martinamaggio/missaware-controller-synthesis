function [sequence] = deadline_stochastic(num_iterations, protected, prob_miss)
% Returns a sequence of length num_iterations where the probability of
% having a miss is prob_miss. The first protected iterations are always
% hits, i.e., = 1.

  sequence = rand(1, num_iterations) > prob_miss; % random generation
  sequence(1:protected) = ones(1, protected);

end