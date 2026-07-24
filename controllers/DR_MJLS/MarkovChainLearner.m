% MARKOVCHAINLEARNER learns the transition probabilities of a Markov chain from data 
% and maintains ambiguity sets for robust control design
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

classdef MarkovChainLearner < handle
% Learns the transition probabilities of a Markov chain from data and 
% maintains ambiguity sets for robust control design.
%
% Properties
%   P               : Estimated transition probability matrix
%   ambiguity_set   : Cell array of AmbiguitySet objects for each mode
%   n               : Number of Markov modes
%   r               : Ambiguity radius for each mode
%   beta            : Confidence level of the ambiguity sets
%
% Methods
%   MarkovChainLearner : Constructor, initializes ambiguity sets and transition matrix
%   update_ambsets     : Updates ambiguity sets and transition matrix using observed mode sequence
    
    properties
        P                   % Estimated transition probability matrix
        ambiguity_set       % Cell array of AmbiguitySet objects for each mode
        n                   % Number of Markov modes
        r                   % Ambiguity radius for each mode
        beta                % Confidence level of the ambiguity sets
    end
    
    methods
        function obj = MarkovChainLearner(n, beta, I_k, p_k, I_v, p_v, l_p)
        % Constructor: Initializes ambiguity sets and transition matrix
        %
        % Inputs
        %   n       : number of modes
        %   beta    : confidence level of the ambiguity sets
        %   I_k     : cell array of indices of known transitions for each mode
        %   p_k     : cell array of values of known transitions for each mode
        %   I_v     : cell array of indices of variable transitions for each mode
        %   p_v     : cell array of bounds of variable transitions for each mode
        %   l_p     : norm used to define ambiguity sets (inf or 1)

            P = zeros(n);
            for i = 1:n
                % Compute mean for varying transitions
                p_v_hat = mean(p_v{i},1);

                % Assign known transition probabilities and mean values for varying transitions
                P(i, I_k{i}) = p_k{i};
                P(i, I_v{i}) = p_v_hat;
                
                % Determine unknown transition probabilities and assign remaining probability mass uniformly
                I_i_u = setdiff([1:n],[I_k{i},I_v{i}]);
                P(i, I_i_u) = (1-sum(p_v_hat,2)-sum(p_k{i}))/length(I_i_u);
            end

            % Create an ambiguity set for each mode
            for i = 1:n
                obj.ambiguity_set{i} = AmbiguitySet(P(i,:), beta, I_k{i}, p_k{i}, I_v{i}, p_v{i}, l_p);
                obj.r(i) = obj.ambiguity_set{i}.r_i;
            end

            obj.n = n;
            obj.beta = beta;
            obj.P = P;
        end

        function [P_hat,r] = update_ambsets(obj, mc)
        % Updates ambiguity sets and transition matrix using observed mode sequence
            
            for i = 1:obj.n
                % find samples of transitions from mode i
                idx_samples = find(mc(1:end-1)==i)+1;
                samples = mc(idx_samples);
                % update ambiguity set of mode i
                [P_i,r_i] = obj.ambiguity_set{i}.update(samples);
                obj.P(i,:) = P_i;
                obj.r(i) = r_i;
            end

            P_hat = obj.P;
            r = obj.r;
        end
    end
end

