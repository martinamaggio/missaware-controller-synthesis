% STOCHASTIC_ANYMISS_GRAPH generates a list of nodes of a minimal graph representing the
% anyMiss(m,k) constraint and a transition probability matrix
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [nodes,tpm] = stochastic_anymiss_graph(m,k,p_miss)
% Generates a list of nodes of a minimal graph representing the
% anyMiss(m,k) constraint and a transition probability matrix for the
% edges (can be used with 'kill' and 'skip' overrun strategy).
%
% Inputs
%   m, k    : parameters of anyMiss(m,k) constraint
%   p_miss  : probability of the next consecutive deadline miss
%
% Outputs
%   nodes   : list of nodes (labels are char arrays of hit/miss sequence)
%   tpm     : transition probability matrix
    
    if isempty(p_miss)
        % set probability of each consecutive miss to 0.5
        p_miss = 0.5*ones(1,m);
    elseif length(p_miss) < m
        % pad p_miss vector for maximum number of consecutive misses
        p_miss = [p_miss, p_miss(end)*ones(1,m-length(p_miss))];
    end

    n_nodes_max = 2^k;      % maximum number of nodes given by window length k
    n_hits_window = k-m;    % minimum number of hits in a window

    nodes_done = cell(1,n_nodes_max);
    tpm = zeros(n_nodes_max);
    % start with node of consecutive hits
    nodes_todo = {repmat('1',1,n_hits_window)};
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
        node_i0 = compact([node_current,'0'],n_hits_window);
        node_i1 = compact([node_current,'1'],n_hits_window);
    
        % if anyMiss(m,k) constraint is not violated and new miss node does 
        % not already exist, add node to nodes_todo
        if sum(node_i0=='0') <= m
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


function node = compact(node,n)
% removes unecessary old symbols and returns the minimal node
    idcs_1 = find(node == '1');
    node = node(idcs_1(end-(n-1)):end);
end