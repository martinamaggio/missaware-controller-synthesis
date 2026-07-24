% METRIC_RMSE computes the root mean squared error results
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function result = metric_rmse(x,u,qoi_name,plant_name,quantile_val)
% returns mean, variance and quantiles of the root mean squared error of x or u
% after finishing the initialization of the system (e.g., the swing-up for the pendulum)
%
% Inputs
%   x               : state trajectory matrix
%   u               : input trajectory matrix
%   qoi_name        : name of quantity of interest ('x' or 'u')
%   plant_name      : name of plant for [plant_name]_enforcedhits.m function
%   quantile_val    : lower quantile to be computed

    init_ongoing = str2func([plant_name,'_enforcedhits']);
    if strcmp(qoi_name,'x')
        qoi = x;
    elseif strcmp(qoi_name,'u')
        qoi = u;
    end
    num_steps = size(x,1);
    result_vec = zeros(size(qoi,3),size(qoi,2));
    for i=1:size(qoi,3)
        init_complete_idx = 1;
        while init_ongoing(x(init_complete_idx,:,i))
            init_complete_idx = init_complete_idx+1;
        end
        result_vec(i,:) = sqrt(1/(num_steps-(init_complete_idx-1)) .* sum(qoi(init_complete_idx:end,:,i).^2,1));
    end
    for i=1:size(qoi,2)
        qoi_name_i = [qoi_name,num2str(i)];
        result.([qoi_name_i,'_mean']) = mean(result_vec(:,i));
        if size(qoi,3)>1
            result.([qoi_name_i,'_var']) = var(result_vec(:,i));
            result.([qoi_name_i,'_quantiles']) = quantile(result_vec(:,i),[quantile_val,1-quantile_val]);
        end
    end
end
