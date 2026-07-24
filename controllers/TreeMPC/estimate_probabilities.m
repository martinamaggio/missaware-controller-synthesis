% ESTIMATE_PROBABILITIES estimates the transition probabilities between nodes
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function tpm = estimate_probabilities(nodes,sequence)
% Estimates the transition probabilities between nodes from a hit/miss sequence.
%
% Inputs
%   nodes       : cell array of nodes
%   sequence    : hit/miss sequence
%
% Outputs
%   tpm         : transition probability matrix

    w = max(cellfun(@(x) length(x),nodes)); % window size
    n = numel(nodes);   % number of nodes

    % translate hit/miss sequence into node sequence
    node_sequence = [];
    for i=1:length(sequence)
        sequence_win = sequence(max(1,i-w+1):i);
        sequence_win = char(strjoin(string(sequence_win),''));
        node_win = find_node(nodes,sequence_win);
        if ~isempty(node_win)
            node_sequence = [node_sequence, node_win];
        end
    end

    tpm = zeros(n,n);

    % compute transition probabilities from each node
    for i=1:n
        % find samples of transitions from node i
        idx_samples = find(node_sequence(1:end-1)==i)+1;
        samples = node_sequence(idx_samples);
        n_samples = length(samples);
        % estimate probability of transitions from node i
        transition_counts = sum(samples(:)==(1:n),1)';
        if n_samples > 0
            tpm(i,:) = transition_counts/n_samples;
        end
    end

end

function i_node = find_node(nodes,sequence)
% finds the node that encodes the end of the sequence and returns its index
    for i=1:length(sequence)
        i_node = find(strcmp(nodes,sequence(i:end)));
        if ~isempty(i_node)
            return
        end
    end
end
