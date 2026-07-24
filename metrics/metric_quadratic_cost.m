% METRIC_QUADRATIC_COST computes the quadratic cost results
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function result = metric_quadratic_cost(x,u,plant_name,quantile_val)
% returns mean, variance and quantiles of a quadratic cost term over x and u
% after finishing the initialization of the system (e.g., the swing-up for the pendulum)
%
% Inputs
%   x               : state trajectory matrix
%   u               : input trajectory matrix
%   plant_name      : name of plant for [plant_name]_enforcedhits.m function
%   quantile_val    : lower quantile to be computed

    init_ongoing = str2func([plant_name,'_enforcedhits']);
    num_steps = size(x,1);
    result_vec = zeros(size(x,3),1);
    Q = eye(size(x,2)); % cost matrix for states
    R = eye(size(u,2)); % cost matrix for inputs
    for i=1:size(x,3)
        init_complete_idx = 1;
        while init_ongoing(x(init_complete_idx,:,i))
            init_complete_idx = init_complete_idx+1;
        end
        cost_sum = 0;
        for t=1:size(x,1)
            cost_sum = cost_sum + x(t,:,i)*Q*x(t,:,i)' + u(t,:,i)*R*u(t,:,i)';
        end
        result_vec(i) = 1/(num_steps-(init_complete_idx-1)) .* cost_sum;
    end
    result.mean = mean(result_vec);
    if size(x,3)>1
        result.var = var(result_vec);
        result.quantiles = quantile(result_vec,[quantile_val, 1-quantile_val]);
    end
end
