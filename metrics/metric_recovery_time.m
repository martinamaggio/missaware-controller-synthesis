% METRIC_RECOVERY_TIME computes the convergence time results
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function result = metric_recovery_time(x,psys)
% returns mean, variance and quantiles of the convergence time
% as the time from the start until the state is sufficiently close to the
% equilibrium (defined by the plant specific function [plant_name]_at_op)
%
% Inputs
%   x               : state trajectory matrix
%   psys            : struct of system and simulation parameters

    close_to_op = str2func([psys.plant_name,'_at_op']);
    result_vec = zeros(size(x,3),1);
    for i=1:size(x,3)
        t = 1;
        while t<size(x,1) && ~close_to_op(x(t,:,i))
            t=t+1;
        end
        result_vec(i) = t * psys.period/psys.sim_factor;
    end
    result.mean = mean(result_vec);
    if size(x,3)>1
        result.var = var(result_vec);
        result.quantiles = quantile(result_vec,[psys.quantile_val,1-psys.quantile_val]);
    end
end