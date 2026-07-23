% STOCHASTIC_ROWMISS_GRAPH generates a list of nodes of a minimal graph representing the
%   rowMiss(m) constraint and a transition probability matrix
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [nodes,tpm] = stochastic_rowmiss_graph(m,p_miss)
% Generates a list of nodes of a minimal graph representing the
% rowMiss(m) constraint and a transition probability matrix for the
% edges (can be used with 'kill' and 'skip' overrun strategy).
%
% Inputs
%   m       : parameters of rowMiss(m) constraint
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

    % the list of nodes contains one hit node and m consecutive miss nodes
    n_nodes = m+1;
    nodes = {'1'};
    tpm = zeros(n_nodes);
    % generate all consecutive miss nodes and determine transition probability
    for i_node = 2:n_nodes
        nodes{i_node} = [nodes{i_node-1},'0'];
        p_miss_current = p_miss(i_node-1);
        tpm(i_node-1,i_node) = p_miss_current;
        tpm(i_node-1,1) = 1-p_miss_current;
    end
    tpm(n_nodes,1) = 1;

end