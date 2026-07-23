% METRIC_COMPUTATION_TIME computes the online computation time results
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function result = metric_computation_time(t,quantile_val)
% returns mean, variance and quantiles of the online computation time
%
% Inputs
%   t               : vector of computation times
%   quantile_val    : lower quantile to be computed

    result = struct('mean',mean(t));
    if size(t,3)>1
        result.var = var(t);
        result.quantiles = quantile(t,[quantile_val,1-quantile_val]);
    end
end

