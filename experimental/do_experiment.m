% DO_EXPERIMENT simulates the current experiment with multiple controllers
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function results = do_experiment(experiment_name,controller_array,eval_methods,psys,prt,process_noise)
% Runs a Monte Carlo simulation of the current experiment consisting of 
% multiple controllers under specific system, simulation and real-time settings, 
% and given sequences of process noise
%
% Inputs
%   experiment_name     : string identifying the experiment
%   controller_array    : cell array of control methods and their
%                         setup functions for the specific experiment
%   eval_methods        : cell array of metrics to be evaluated
%   psys                : struct with system and simulation parameters
%   prt                 : struct with the hit/miss sequence (1 x nt x nmc)
%   process_noise       : array of process noise (nx x nt x nmc)
%
% Outputs
%   results             : evaluation of given metrics

    % extract the needed system and real-time parameters
    plant_name = psys.plant_name;   % plant name
    period = psys.period;           % sample period
    hm_sequence = prt.hm_sequence;  % hit/miss sequence
    nmc = psys.n_mc;                % number of Monte Carlo simulation runs
    p_quant = psys.quantile_val;    % lower quantile to be plotted if nmc>1

    set(groot,'defaulttextinterpreter','none')
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    save_path = fullfile(repo_root, 'data', plant_name, experiment_name);
    if ~isfolder(save_path)
        mkdir(save_path)
    end

    num_ctrl = numel(controller_array);
    results = cell(numel(controller_array),1);
    t_seq = (0:size(hm_sequence,2)-1)*period;
    
    f = figure('WindowState','maximized');
    til = tiledlayout(f,2,num_ctrl+1,'TileIndexing','columnmajor','Padding','compact','TileSpacing','compact');
    title(til,experiment_name,'interpreter','none');

    for i=1:num_ctrl
        current_method = controller_array{i}.method;
        disp(['Setting up and simulating ',current_method,'...']);
        t_offline_start = tic;
        [current_controller, psys.z0] = controller_array{i}.function();
        t_eval = toc(t_offline_start);
        results{i} = struct('method',current_method);
    
        for j=1:nmc
            if size(hm_sequence,3) > 1
                current_hm_sequence = hm_sequence(:,:,j);
            else
                current_hm_sequence = hm_sequence;
            end
            if size(process_noise,3) > 1
                current_process_noise = process_noise(:,:,j);
            else
                current_process_noise = process_noise;
            end
            % perform the simulation
            [t, x_mc{j}, y_mc{j}, z_mc{j}, u_mc{j}, t_ctrl] = simulate(psys, prt, ...
                current_controller, current_hm_sequence, current_process_noise);
            t_eval = [t_eval; t_ctrl];
        end

        x = cat(3,x_mc{:});
        u = cat(3,u_mc{:});

        metric_cell_array = cell(numel(eval_methods),1);
        for k=1:numel(eval_methods)
            current_metric = eval_methods{k}.function(x,u,t_eval);
            current_metric.name = eval_methods{k}.name;
            metric_cell_array{k} = current_metric;
        end
        results{i}.metrics = metric_cell_array;


        x_mean = mean(x,3);
        u_mean = mean(u,3);

        x_lb = [];
        x_ub = [];
        u_lb = [];
        u_ub = [];
        if nmc>1
            x_lb = quantile(x,p_quant,3);
            x_ub = quantile(x,1-p_quant,3);
            u_lb = quantile(u,p_quant,3);
            u_ub = quantile(u,1-p_quant,3);
        end

        feval([plant_name,'_plot'],f,til,current_method,t,u_mean,u_lb,u_ub,x_mean,x_lb,x_ub);
        figure(f)
        
        filename_csv = [save_path,'/',current_method,'.csv'];
        header = {'t'};
        data_step = 20;
        data = {t(1:data_step:end)};
        for j = 1:size(x,2)
            header = [header, {['x',num2str(j)]}];
            data = [data, {x_mean(1:data_step:end,j)}];
        end
        for j = 1:size(u,2)
            header = [header, {['u',num2str(j)]}];
            data = [data, {u_mean(1:data_step:end,j)}];
        end
        header = [header, {'sequence'}];
        data = [data, {hm_sequence(1,:,1)'}];

        if nmc>1
            for j = 1:size(x,2)
                header = [header, {['xlb',num2str(j)]}];
                data = [data, {x_lb(1:data_step:end,j)}];
                header = [header, {['xub',num2str(j)]}];
                data = [data, {x_ub(1:data_step:end,j)}];
            end
            for j = 1:size(u,2)
                header = [header, {['ulb',num2str(j)]}];
                data = [data, {u_lb(1:data_step:end,j)}];
                header = [header, {['uub',num2str(j)]}];
                data = [data, {u_ub(1:data_step:end,j)}];
            end
        end

        table_to_save = array2table(cell2mat(data),'VariableNames',header);
        writetable(table_to_save,filename_csv);
    end
    
    nexttile(til);
    plot(t_seq, current_hm_sequence, '*'); grid on;
    legend("deadline hit/miss");
    
    yticks([0 1]);
    yticklabels({'Miss','Hit'});
    ylim([-0.5 1.5]);
    grid on
    
    title('deadline hit/miss sequence')
    xlabel('time');
    
    if ~isempty(eval_methods)
        display_evaluation(results);
    end
end

function display_evaluation(results)
    for i=1:numel(results)
        disp([results{i}.method,':']);
        for j=1:numel(results{i}.metrics)
            fn = fieldnames(results{i}.metrics{j});
            string_display = ['   ',results{i}.metrics{j}.name,' - '];
            for jj = 1:numel(fn)-1
                fn_jj = fn{jj};
                output_data = results{i}.metrics{j}.(fn_jj);
                if length(output_data) > 1
                    output_data_str = sprintf(['[',repmat(' %.3f',1,numel(output_data)),' ]'],output_data);
                    
                else
                    output_data_str = num2str(results{i}.metrics{j}.(fn_jj),3);
                end
                string_display = [string_display,fn_jj,': ',output_data_str,'  '];
            end
            disp(string_display);
        end
        disp(' ');
    end
end