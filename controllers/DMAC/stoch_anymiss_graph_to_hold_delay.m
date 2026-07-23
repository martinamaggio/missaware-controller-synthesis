% STOCH_ANYMISS_GRAPH_TO_HOLD_DELAY computes probabilistic delay and hold values 
% for DMAC based on a stochastic anyMiss(m,k) graph
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [delay_mat,hold_mat] = stoch_anymiss_graph_to_hold_delay(nodes,tpm,overrun_strategy,actuator_strategy)
% Computes probabilistic delay and hold values for DMAC based on a
% stochastic anyMiss(m,k) graph.
%
% Inputs
%   nodes               : cell array of nodes
%   tpm                 : transition probabilities between nodes
%   overrun_strategy    : overrun strategy ('kill', 'skip')
%   actuator_strategy   : actuator strategy ('hold', 'zero')
%
% Outputs
%   delay_mat           : probabilities of different delay values
%   hold_mat            : probabilities of different hold values

    % compute stationary distribution from transition probability matrix
    [V,D] = eig(tpm');
    v_stat = V(:,abs(diag(D)'-1)<1e-6);
    stationary_dist = v_stat./sum(v_stat);
    
    % compute the probabilities of paths of different lengths starting and 
    % ending with a hit node
    [paths,prob_mat] = compute_path_probabilities(nodes, tpm);
    length_vec = 1:size(prob_mat,2);

    % compute the probabilities of the different lengths as the stationary
    % probability of being in a specific node multiplied by the
    % probabilities of different length paths from this node
    probs_lengths = sum(stationary_dist.*prob_mat,1);
    % normalize the probabilities since only hit nodes are relevant
    probs_normalized = probs_lengths/sum(probs_lengths);
    hold_mat = [probs_normalized(:),length_vec(:)];

    % delay values depend on overrun strategy
    switch(overrun_strategy)
        case 'kill'
            switch(actuator_strategy)
                case 'zero' % Either one active control input (in case of HH) or zero
                    if (sum(endsWith(nodes,'11'))> 0) 
                        prob_11 = sum(stationary_dist(endsWith(nodes,'11')))/ ...
                            sum(stationary_dist(endsWith(nodes,'1')));
                    else 
                        prob_11 = stationary_dist(endsWith(nodes,'1'))* tpm(1,1);
                    end
                    delay_mat = [prob_11, 1; 1-prob_11, 1];
                case 'hold'
                    delay_mat = [1, 1];
                otherwise
                    error('invalid actuator strategy')
            end            
        case 'skip'
            delay_mat = hold_mat;
        otherwise
            error('invalid overrun strategy')
    end
end


function [paths,prob_mat] = compute_path_probabilities(nodes,tpm)
% compute the probabilities of paths of different lengths starting and 
% ending with a hit node

    % maximum path length bounded by maximum number of misses
    max_path_length = max(cellfun(@(x) sum(x=='0'),nodes))+1;
    
    num_nodes = length(nodes);
    ends_in_1 = false(num_nodes, 1);
    for i = 1:num_nodes
        ends_in_1(i) = nodes{i}(end) == '1';
    end
    
    paths = [];
    prob_mat = zeros(num_nodes,max_path_length);

    % bfs algorithm starting from each node that ends in '1'
    for start_idx = 1:num_nodes
        if ~ends_in_1(start_idx)
            continue;
        end
                        
        queue = {start_idx, 1.0, 0, start_idx};
        while ~isempty(queue)
            current_idx = queue{1};
            current_prob = queue{2};
            current_length = queue{3};
            current_path = queue{4};
            queue(1:4) = [];
            
            if current_length >= max_path_length
                continue;
            end
            
            % explore all neighbors
            for next_idx = 1:num_nodes
                if tpm(current_idx, next_idx) > 0
                    new_prob = current_prob * tpm(current_idx, next_idx);
                    new_length = current_length + 1;
                    new_path = [current_path, next_idx];
                    
                    if ends_in_1(next_idx)
                        prob_mat(start_idx,new_length) = new_prob;
                        paths = [paths, struct('path_idcs',new_path,'path_prob',new_prob,'path_length',new_length)];
                    else
                        queue{end+1} = next_idx;
                        queue{end+1} = new_prob;
                        queue{end+1} = new_length;
                        queue{end+1} = new_path;
                    end
                end
            end
        end
    end
end

