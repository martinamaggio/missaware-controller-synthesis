% RUN_CASE_STUDY_2 runs the electric motor with increasing upper bounds of the control task's response time
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

% Figures 6 - 7 of the paper are created using the data generated when running this script. 
% This runs the open-loop system and the output-feedback deadline-miss-aware 
% controllers and their nominal baseline controller with increasing upper bounds 
% of the control task's response time on an electric motor.

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
psys.C = [1,0,0;
          0,0,1];                               % output matrix (i_d and w_el are measured)
psys.sim_factor = 20;                           % upsampling factor of controller period for simulation
psys.quantile_val = 0.1;

duration = 0.3;                                 % duration of simulation (seconds)
num_iterations = ceil(duration/psys.period);    % number of discrete time steps of simulation
prt.wh_constraint = 'any_miss';                 % type of weakly hard constraint
num_mc = 200;                                   % number of Monte Carlo runs

process_noise_std_dev = [0.1; 0.1; 0.2];
process_noise_seq = process_noise_std_dev .* randn(length(psys.x0),num_iterations,num_mc);

kick_seq = zeros(length(psys.x0),num_iterations);
kick_seq_idcs = 4;
kick_seq(3,kick_seq_idcs,:) = -10;

% list of metrics that are computed for all controllers
metrics = {
    struct('name', 'convergence_time', 'function', @(x,u,t) metric_recovery_time(x(kick_seq_idcs(1)*psys.sim_factor:end,:,:),psys));
    struct('name', 'offline_computation_time', 'function', @(x,u,t) struct('value',t(1)));
    struct('name', 'online_computation_time', 'function', @(x,u,t) metric_computation_time(t(2:end),psys.quantile_val));
    };

%% StreamAdapt vs. DMAI vs. Baseline Output Feedback with varying response time distributions
experiment_name = 'output_feedback';
rt_low = psys.period*0.1;
rt_high_vals = psys.period*[1:0.25:5];

prt_kill = prt;
prt_skip = prt;
prt_kill.overrun_strategy = 'kill';
prt_kill.actuator_strategy = 'hold';
prt_skip.overrun_strategy = 'skip';
prt_skip.actuator_strategy = 'hold';

results_kill = [];
results_skip = [];
for i=1:length(rt_high_vals)
    if rt_high_vals(i) <= psys.period
        psys.n_mc = 1;
    else
        psys.n_mc = num_mc;
    end
    rt_dist = @(n) unifrnd(rt_low, rt_high_vals(i), n, 1);
    prt_kill.hm_sequence = zeros(1,num_iterations,psys.n_mc);
    prt_skip.hm_sequence = zeros(1,num_iterations,psys.n_mc);
    m_max_skip = 0;
    m_max_kill = 0;
    for j=1:psys.n_mc
        [prt_kill.hm_sequence(:,:,j),mj] = sample_rt_distribution(num_iterations,prt_kill.overrun_strategy,psys.period,rt_dist,[]);
        if mj>m_max_kill
            m_max_kill = mj;
        end
        [prt_skip.hm_sequence(:,:,j),mj] = sample_rt_distribution(num_iterations,prt_skip.overrun_strategy,psys.period,rt_dist,[]);
        if mj>m_max_skip
            m_max_skip = mj;
        end
    end
    disp(['Kill: rowMiss(',num2str(m_max_kill),')'])
    disp(['Skip-next: rowMiss(',num2str(m_max_skip),')'])
    prt_skip.m = m_max_skip;
    % list of controllers that are simulated with the generated hit/miss sequences
    controllers_kill = {
        struct('method', 'open_loop', 'function', @() deal(@ctrl_none, []));
        struct('method', 'dmai_kill', 'function', @() setup_dmai(psys,prt_kill,@ctrl_motor_output));
        struct('method', 'nominal_kill', 'function', @() setup_nominal_motor_ctrl(psys.period,psys.C,@ctrl_motor_output))
        };
    controllers_skip = {
        struct('method', 'streamadapt_skip', 'function', @() setup_streamadapt(psys,prt_skip,'output'));
        struct('method', 'nominal_skip', 'function', @() setup_nominal_motor_ctrl(psys.period,psys.C,@ctrl_motor_output))
        };

    results_kill=[results_kill, do_experiment(['output_feedback/',num2str(i)],controllers_kill,metrics,psys,prt_kill,kick_seq)];
    results_skip=[results_skip, do_experiment(['output_feedback/',num2str(i)],controllers_skip,metrics,psys,prt_skip,kick_seq)];
end
results = [results_kill; results_skip];
controllers = [controllers_kill; controllers_skip];

%% Data processing and visualization
save_path = ['data/',psys.plant_name,'/',experiment_name];
figure_title = 'Means of convergence time';
process_data(save_path,figure_title,results,controllers,rt_high_vals,psys.period);

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

function process_data(save_path,figure_title,results,controllers,bar_t,T)
    num_ctrl = numel(controllers);
    num_points = size(results,2);
    
    header = {'points'};
    
    mean_convergence_time = zeros(num_points,num_ctrl);
    for idx_ctrl=1:num_ctrl
        for idx_point=1:num_points
            mean_convergence_time(idx_point,idx_ctrl) = extract_kpi(results(:,idx_point),idx_ctrl,1);
        end
    end
    
    header = [header, {'mean_convergence_time'}];
    data_ctrls = mean_convergence_time;
    
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
    
        data = {bar_t(:)};
    
        for i=1:size(data_ctrls,3)
            data = [data, {data_ctrls(:,idx_ctrl,i)}];
        end
    
        table_to_save = array2table(cell2mat(data),'VariableNames',header);
        writetable(table_to_save,filename_csv);
    end
    
    figure
    plot(bar_t(1:size(results,2))./T,mean_convergence_time(:,1),'DisplayName','open loop','Color','black','LineWidth',0.2);
    hold on; grid on;
    for i=2:numel(controllers)
        ctrl_name = controllers{i}.method;
        plot(bar_t(1:size(results,2))./T,mean_convergence_time(:,i),'DisplayName',regexprep(ctrl_name,'\_','\\_'),'LineWidth',1);
    end
    title(figure_title)
    xlabel('$\bar{t}/T$','interpreter','latex')
    legend  
    ylim([0.0,0.2])

end