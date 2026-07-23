% RUN_CASE_STUDY_1 runs the electric motor with varying values of the deadline miss probability
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

% Figures 4 - 5 of the paper are created using the data generated when running this script. 
% This runs the open-loop system and the selected controllers under kill/zero 
% and skip/zero strategies with varying values of p on an electric motor.

clear; clc;
close all;
rng(1);
tic

addpath(genpath(pwd));

% parameters of the simulation *******************************************
psys.plant_name = 'motor';                      % plant {'motor', 'furuta'}
psys.plant_sim = 'nonlinear';                   % nonlinear or linear model for simulation
psys.period = 0.001;                            % controller period (seconds)
psys.x0 = [0; 0; 0];                            % initial state (i_d, i_q, w_el)
psys.u0 = [0; 0];                               % initial control signal (u_d, u_q)
psys.C = eye(length(psys.x0));                  % output matrix (full state feedback)

duration = 0.1;                                 % duration of simulation (seconds)
num_iterations = ceil(duration/psys.period);    % number of discrete time steps of simulation
prt.wh_constraint = 'any_miss';                 % type of weakly hard constraint
num_mc = 200;                                   % number of Monte Carlo runs 
psys.quantile_val = 0.1;                        % quantiles to be computed for states, inputs, metrics
prt.m = 4;                                      % m,h,w: parameters of wh constraint 
prt.w = 5;                                      %        (example: anyMiss(m,w) or rowHit(h,w))
prt.h = prt.w-prt.m;
p_miss_vals = [0.0:0.1:1.0];                    % values of p_miss 
                                                % (probability of a deadline miss for free iterations)

process_noise_std_dev = [0.2; 0.2; 0.2];        % standard deviation of process noise for each state
process_noise_seq = process_noise_std_dev .* randn(length(psys.x0),num_iterations,num_mc);
kick_seq = zeros(length(psys.x0),num_iterations,num_mc);
kick_seq_idcs = 4;                              % indices of offset disturbance
kick_seq(3,kick_seq_idcs,:) = -10*ones(1,length(kick_seq_idcs),num_mc);

% list of metrics that are computed for all controllers
metrics = {
    struct('name', 'quadratic_cost', 'function', @(x,u,t) metric_quadratic_cost(x,u,psys.plant_name,psys.quantile_val));
    struct('name', 'offline_computation_time', 'function', @(x,u,t) struct('value',t(1)));
    struct('name', 'online_computation_time', 'function', @(x,u,t) metric_computation_time(t(2:end),psys.quantile_val));
    };

switch prt.wh_constraint
    case 'any_miss'
        wh_constraint_name = ['anyMiss(',num2str(prt.m),',',num2str(prt.w),')'];
    case 'any_hit'
        wh_constraint_name = ['anyHit(',num2str(prt.h),',',num2str(prt.w),')'];
    case 'row_miss'
        wh_constraint_name = ['rowMiss(',num2str(prt.m),')'];
    case 'row_hit'
        wh_constraint_name = ['rowHit(',num2str(prt.h),',',num2str(prt.w),')'];
end

%% Kill/zero controllers
experiment_name = 'varying_p_miss_kill_zero';
prt.overrun_strategy = "kill";
prt.actuator_strategy = "zero";

results_kz = [];
for i=1:length(p_miss_vals)
    p_miss = p_miss_vals(i);
    if p_miss==0.0 || p_miss==1.0
        psys.n_mc = 1;                  % no deadline misses or all deadline misses -> deterministic simulation
    else
        psys.n_mc = num_mc;             % deadline misses are probabilistic -> Monte Carlo simulation
    end
    % generate hit/miss sequences for the given anyMiss(m,w) constraint and
    % the respective p_miss value
    prt.hm_sequence = zeros(1,num_iterations,psys.n_mc);
    for j=1:psys.n_mc
        prt.hm_sequence(:,:,j) = deadline_anymiss(num_iterations,0,prt.m,prt.w,p_miss);
    end
    % generate a sample hit/miss sequence to estimate the overall
    % probabilities for the stochastic methods
    prt.sample_sequence = deadline_anymiss(1e5,0,prt.m,prt.w,p_miss);
    % empty sample hit/miss sequence for the robust version of DR-MJLS
    prt_rob = prt;
    prt_rob.sample_sequence = [];
    % list of controllers that are simulated with the generated hit/miss sequences
    controllers_kz = {
        struct('method', 'open_loop', 'function', @() deal(@ctrl_none, []));
        struct('method', 'graph_lqr', 'function', @() setup_graph_lqr(psys,prt));
        struct('method', 'dmac', 'function', @() setup_dmac(psys,prt));
        struct('method', 'dr_mjls', 'function', @() setup_drmjls(psys,prt,true));
        struct('method', 'tree_mpc', 'function', @() setup_treempc(psys,prt));
        struct('method', 'gss', 'function', @() setup_gss(psys,prt,true));
        };
    results_kz=[results_kz, do_experiment([experiment_name,'/',num2str(i)],controllers_kz,metrics,psys,prt,kick_seq)];
end

%% Data processing and visualization
save_path = ['data/',psys.plant_name,'/',experiment_name];
figure_title = [wh_constraint_name,' - Kill/Zero - Open-loop-normalized mean of quadratic cost'];
process_data(save_path,figure_title,results_kz,controllers_kz,p_miss_vals);


%% Skip/zero controllers
experiment_name = 'varying_p_miss_skip_zero';
prt.overrun_strategy = "skip";
prt.actuator_strategy = "zero";

results_sz = [];
for i=1:length(p_miss_vals)
    p_miss = p_miss_vals(i);
    if p_miss==0.0 || p_miss==1.0
        psys.n_mc = 1;                  % no deadline misses or all deadline misses -> deterministic simulation
    else
        psys.n_mc = num_mc;             % deadline misses are probabilistic -> Monte Carlo simulation
    end
    % generate hit/miss sequences for the given anyMiss(m,w) constraint and
    % the respective p_miss value
    prt.hm_sequence = zeros(1,num_iterations,psys.n_mc);
    for j=1:psys.n_mc
        prt.hm_sequence(:,:,j) = deadline_anymiss(num_iterations,0,prt.m,prt.w,p_miss);
    end
    % generate a sample hit/miss sequence to estimate the overall
    % probabilities for the stochastic methods
    prt.sample_sequence = deadline_anymiss(1e5,0,prt.m,prt.w,p_miss);
    % empty sample hit/miss sequence for the robust version of DR-MJLS
    prt_rob = prt;
    prt_rob.sample_sequence = [];
    % list of controllers that are simulated with the generated hit/miss sequences
    controllers_sz = {
        struct('method', 'open_loop', 'function', @() deal(@ctrl_none, []));
        struct('method', 'dmac', 'function', @() setup_dmac(psys,prt));
        struct('method', 'dr_mjls', 'function', @() setup_drmjls(psys,prt,true));
        struct('method', 'tree_mpc', 'function', @() setup_treempc(psys,prt));
        struct('method', 'gss', 'function', @() setup_gss(psys,prt,true));
        };
    results_sz=[results_sz, do_experiment([experiment_name,'/',num2str(i)],controllers_sz,metrics,psys,prt,kick_seq)];
end


%% Data processing and visualization
save_path = ['data/',psys.plant_name,'/',experiment_name];
figure_title = [wh_constraint_name,' - Skip/Zero - Open-loop-normalized mean of quadratic cost'];
process_data(save_path,figure_title,results_sz,controllers_sz,p_miss_vals);

toc

%% Functions
function [means,vars,quantiles] = extract_kpi(results,method_idx,metric_idx)
    metric_names = fieldnames(results{method_idx}.metrics{metric_idx});
    names_means = metric_names(cellfun(@(x) endsWith(x,'mean'),metric_names));
    n_means = numel(names_means);

    names_vars = metric_names(cellfun(@(x) endsWith(x,'var'),metric_names));
    n_vars = numel(names_vars);

    names_quants = metric_names(cellfun(@(x) endsWith(x,'quantiles'),metric_names));
    n_quants = numel(names_quants);

    if n_means == 0
        means = results{method_idx}.metrics{metric_idx}.value;
    end
    for i=1:n_means
        means(i,1) = results{method_idx}.metrics{metric_idx}.(names_means{i});
    end
    for i=1:n_vars
        vars(i,1) = results{method_idx}.metrics{metric_idx}.(names_vars{i});
    end
    for i=1:n_quants
        quantiles(i,1:2) = results{method_idx}.metrics{metric_idx}.(names_quants{i});
    end
end

function process_data(save_path,figure_title,results,controllers,p_miss_vals)
    num_ctrl = numel(controllers);
    num_points = size(results,2);
    
    header = {'points'};
    
    mean_quadratic_cost_to_ol = zeros(num_points,num_ctrl);
    quadratic_cost_ol = extract_kpi(results(:,1),1,1);
    
    for idx_ctrl=1:num_ctrl
        mean_quadr_costs = zeros(num_points,1);
        for idx_point=1:num_points
            mean_quadr_costs(idx_point) = extract_kpi(results(:,idx_point),idx_ctrl,1);
        end
        mean_quadratic_cost_to_ol(:,idx_ctrl) = mean_quadr_costs ./ quadratic_cost_ol;
    end
    
    header = [header, {'mean_quadratic_cost_to_ol'}];
    data_ctrls = mean_quadratic_cost_to_ol;
    
    mean_computation_times = zeros(num_points,num_ctrl,2);
    for idx_metric = 2:3
        for idx_ctrl=1:num_ctrl
            mean_comp_times = zeros(num_points,1);
            for idx_point=1:num_points
                mean_comp_times(idx_point) = extract_kpi(results(:,idx_point),idx_ctrl,idx_metric);
            end
            mean_computation_times(:,idx_ctrl,idx_metric-1) = mean_comp_times;
        end
    end
    
    header = [header, {'offline_computation_time','mean_online_computation_time'}];
    data_ctrls = cat(3,data_ctrls,mean_computation_times(:,:,1),mean_computation_times(:,:,2));
    
    for idx_ctrl = 1:num_ctrl
        ctrl_name = controllers{idx_ctrl}.method;
        filename_csv = [save_path,'/metrics_',ctrl_name,'.csv'];
    
        data = {p_miss_vals(:)};
    
        for i=1:size(data_ctrls,3)
            data = [data, {data_ctrls(:,idx_ctrl,i)}];
        end
    
        table_to_save = array2table(cell2mat(data),'VariableNames',header);
        writetable(table_to_save,filename_csv);
    end
    
    figure
    for i=2:numel(controllers)
        ctrl_name = controllers{i}.method;
        plot(p_miss_vals(1:size(results,2)),mean_quadratic_cost_to_ol(:,i),'DisplayName',regexprep(ctrl_name,'\_','\\_'));
        hold on; grid on;
    end
    title(figure_title)
    xlabel('$p$','interpreter','latex')
    legend  
    ylim([0.2,1.1])

end