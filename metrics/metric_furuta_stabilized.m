% METRIC_FURUTA_STABILIZED computes the results on the amount of time the pendulum is stabilized
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function result = metric_furuta_stabilized(x)
% returns mean and variance of the ratio of time steps the pendulum is
% close to the upright equilibrium (meaning pendulum angle < 0.2 rad)
% after finishing the swing-up
%
% Inputs
%   x : state trajectory matrix

    num_steps = size(x,1);
    result_vec = zeros(size(x,3),1);
    for i=1:size(x,3)
        swingup_complete = find(abs(x(:,1,i))<0.5,1);
        result_vec(i) = 1/(num_steps-swingup_complete) .* sum(abs(x(swingup_complete:end,1,i))<0.2);
    end
    result = struct('mean',mean(result_vec),'var',var(result_vec));
end

