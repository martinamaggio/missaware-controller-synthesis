% STOCHASTIC_ROWHIT_GRAPH generates a list of nodes of a minimal graph representing the
% rowHit(m,k) constraint and a transition probability matrix
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [nodes,tpm] = stochastic_rowhit_graph(m,k,p_miss)
% Generates a list of nodes of a minimal graph representing the
% rowHit(m,k) constraint and a transition probability matrix for the
% edges (can be used with 'kill' and 'skip' overrun strategy).
%
% Inputs
%   m, k    : parameters of rowHit(m,k) constraint
%   p_miss  : probability of the next consecutive deadline miss
%
% Outputs
%   nodes   : list of nodes (labels are char arrays of hit/miss sequence)
%   tpm     : transition probability matrix
    
    n_miss_max = k-m;
    if isempty(p_miss)
        % set probability of each consecutive miss to 0.5
        p_miss = 0.5*ones(1,n_miss_max);
    elseif length(p_miss) < n_miss_max
        % pad p_miss vector for maximum number of consecutive misses
        p_miss = [p_miss, p_miss(end)*ones(1,n_miss_max-length(p_miss))];
    end

    n_nodes_max = 2^k;      % maximum number of nodes given by window length k

    nodes_done = cell(1,n_nodes_max);
    tpm = zeros(n_nodes_max);
    % start with node of consecutive hits
    nodes_todo = {repmat('1',1,m)};
    i_node = 0;

    while ~isempty(nodes_todo)
        % move current node from nodes_todo to nodes
        i_node = i_node+1;
        node_current = nodes_todo{1};
        nodes_todo(1) = [];
        nodes_done{i_node} = node_current;

        % identify relevant miss probability by number of consecutive misses
        n_trailing_zeros = length(node_current) - find(node_current=='1',1,'last');
        i_p_miss = n_trailing_zeros + 1;
        
        % potential neighbouring nodes
        node_i0 = compact([node_current,'0'],m,k);
        node_i1 = compact([node_current,'1'],m,k);
    
        % if rowHit(m,k) constraint is not violated and new miss node does 
        % not already exist, add node to nodes_todo
        if satisfies_rowhit(node_i0,m) && future_rowhit_possible(node_i0,m,k)
            i_node_i0 = find(strcmp(nodes_done,node_i0));
            if isempty(i_node_i0)
                nodes_todo(end+1) = {node_i0};
                i_node_i0 = i_node+numel(nodes_todo);
            end
            % set transition probability to probability of a miss from
            % current node
            p_miss_current = p_miss(i_p_miss);
            tpm(i_node,i_node_i0) = p_miss_current;
        else
            % if anyMiss(m,k) constraint would be violated, a hit is enforced
            p_miss_current = 0;
        end
        % add new hit node to nodes_todo if it does not already exist 
        i_node_i1 = find(strcmp(nodes_done,node_i1));
        if isempty(i_node_i1)
            nodes_todo(end+1) = {node_i1};
            i_node_i1 = i_node+numel(nodes_todo);
        end
        % the transition probability is the complement of the miss probability
        tpm(i_node,i_node_i1) = 1-p_miss_current;
    end
    % return the minimal graph nodes and trimmed transition probability matrix
    nodes = nodes_done(1:i_node);
    tpm = tpm(1:i_node,1:i_node);
end

function [lengths, ends] = get_hit_sequences(node)
% extracts the lengths of all sequences of consecutive hits and their end indices
    seq_changes = diff(['0',node,'0']);
    starts = find(seq_changes == 1);  % hit sequences start when the sequence changes from 0 to 1
    ends = find(seq_changes == -1);   % hit sequences end when the sequence changes from 1 to 0
    lengths = ends-starts;
    if isempty(lengths)
        lengths = 0;
    end
end

function node = compact(node,n_hits,win_size)
% removes unecessary old symbols before the newest fulfillment of the rowHit
% constraint and returns the minimal node truncated to the window size
    [lengths, ends] = get_hit_sequences(node);
    i_rowhit_fulfilled = find(lengths>=n_hits,1,'last');
    % shorten the node so its sequence starts with fulfilling the rowHit constraint
    node = node(ends(i_rowhit_fulfilled)-n_hits:end);
    if length(node) > win_size
        node = node(length(node)-win_size+1:end);
    end
end

function res = satisfies_rowhit(node,n_hits,win_size)
% checks if current node satisfies rowHit constraint
    lengths = get_hit_sequences(node);
    res = max(lengths)>=n_hits && length(node)<=win_size;
end

function res = future_rowhit_possible(node,n_hits,win_size)
% checks if rowHit constraint can still be fulfilled in the future starting
% at the current node
    node_next_hit = node;
    for i=1:win_size
        node_next_hit = [node_next_hit,'1'];
        if length(node_next_hit) > win_size
            node_next_hit = node_next_hit(length(node)-win_size+1:end);
        end
        if ~satisfies_rowhit(node_next_hit,n_hits)
            res = false;
            return
        end
    end
    res = true;
end