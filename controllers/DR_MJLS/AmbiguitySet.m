% AMBIGUITYSET represents an ambiguity set for a row of a Markov transition matrix
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

classdef AmbiguitySet < handle
% Represents an ambiguity set for a row of a Markov transition matrix.
%
% Properties
%   P_i_hat     : Estimated transition probabilities
%   gamma_i_u   : Inverse of mode-specific sample size of unknown transitions
%   r_i         : Ambiguity radius
%   r_i_v       : Ambiguity radius for varying transitions
%   beta        : Confidence level of the ambiguity set
%   k_i         : Number of known transitions
%   u_i         : Number of unknown static transitions
%   v_i         : Number of varying transitions
%   I_i_k       : Indices of known transitions
%   p_ij_k      : Probability values of known transitions
%   p_i_k       : Total probability mass of known transitions
%   I_i_v       : Indices of varying transitions
%   p_ij_v      : Probability bounds of varying transitions
%   p_ij_v_mean : Means of varying transitions
%   p_i_v_min   : Minimum total probability mass of varying transitions
%   I_i_u       : Indices of unknown static transitions
%   p_i_u_hat   : Estimated total probability mass of unknown transitions
%   p_ij_cond_u : Estimated conditional probabilities for unknown transitions
%   l_p         : Norm used for the ambiguity set
%
% Methods
%   AmbiguitySet : Constructor
%   get_radius   : Computes ambiguity radius based on confidence and sample size
%   update       : Updates ambiguity set and estimated probabilities using new samples

    properties
        P_i_hat     % Estimated transition probabilities
        gamma_i_u   % Inverse of mode-specific sample size of unknown transitions
        r_i         % Ambiguity radius
        r_i_v       % Ambiguity radius for varying transitions
        beta        % Confidence level of the ambiguity set
        k_i         % Number of known transitions
        u_i         % Number of unknown static transitions
        v_i         % Number of varying transitions
        I_i_k       % Indices of known transitions
        p_ij_k      % Probability values of known transitions
        p_i_k       % Total probability mass of known transitions
        I_i_v       % Indices of varying transitions
        p_ij_v      % Probability bounds of varying transitions
        p_ij_v_mean % Means of varying transitions
        p_i_v_min   % Minimum total probability mass of varying transitions
        I_i_u       % Indices of unknown static transitions
        p_i_u_hat   % Estimated total probability mass of unknown transitions
        p_ij_cond_u % Estimated conditional probabilities p_(ij|U) for unknown transitions
        l_p         % Norm used for the ambiguity set
    end
    
    methods
        function obj = AmbiguitySet(P, beta, I_k, p_k, I_v, p_v, l_p)
        % Constructor
        %
        % Inputs
        %   P       : Estimated transition probabilities
        %   beta    : Confidence level
        %   I_k     : Indices of known transitions
        %   p_k     : Probability values of known transitions
        %   I_v     : Indices of varying transitions
        %   p_v     : Probability bounds of varying transitions
        %   l_p     : Norm used for the ambiguity set (inf or 1)

            % Initialize properties based on inputs
            obj.P_i_hat = P;
            obj.beta = beta;
            obj.I_i_k = I_k;
            obj.p_ij_k = p_k;
            obj.p_i_k = sum(p_k);
            obj.I_i_v = I_v;
            obj.gamma_i_u = 100; % initialize inverse sample size to a large value (no samples yet)
            obj.l_p = l_p;
            obj.k_i = length(I_k);
            obj.v_i = length(I_v);
            obj.u_i = length(P) - obj.k_i - obj.v_i;
            obj.p_ij_v = p_v;
            if obj.v_i ~= 0
                % Compute ambiguity radius and mean for varying transitions
                obj.r_i_v = 0.5 * max(abs(p_v(2,:)-p_v(1,:)));
                obj.p_ij_v_mean = mean(p_v,1);
                obj.p_i_v_min = sum(p_v(1,:));
            else
                obj.r_i_v = 0;
                obj.p_ij_v_mean = 0;
                obj.p_i_v_min = 0;
            end

            % Indices and estimated total probability mass of unknown transitions
            obj.I_i_u = setdiff([1:length(P)],[I_k,I_v]);
            obj.p_i_u_hat = 1 - obj.p_i_k - sum(obj.p_ij_v_mean);
            % Initialize conditional probabilities for unknown transitions
            obj.p_ij_cond_u = zeros(1,length(P));
            obj.p_ij_cond_u(1,obj.I_i_u) = 1/length(obj.I_i_u);
            
            % Compute ambiguity radius
            obj.r_i = 100;
            obj.r_i = get_radius(obj);

        end


        function r = get_radius(obj)
        % Compute ambiguity radius for the set based on confidence and sample size

            % Use varying transition radius as a lower bound
            if obj.r_i <= obj.r_i_v
                r = obj.r_i_v;
                return
            end      

            b = obj.beta;
            n_uv = obj.v_i + obj.u_i; % total number of unknown and varying transitions
            
            % Compute c: contribution from varying transitions
            if obj.v_i > 0
                c = max(obj.p_ij_cond_u) * 0.5 * sum(obj.p_ij_v(2,:)-obj.p_ij_v(1,:),2);
            else
                c = 0;
            end

            % Compute p_u_max: max probability mass for unknown transitions
            if obj.u_i > 0
                p_u_max = 1 - obj.p_i_k - obj.p_i_v_min;
            else
                p_u_max = 0;
                c = 0;
            end

            % Compute ambiguity radius based on norm
            switch obj.l_p
                case 1
                    r = 2 * n_uv * p_u_max * sqrt(log(2/b) / (2*obj.gamma_i_u^(-1))) + n_uv * c;
                case inf
                    r = 2 * p_u_max * sqrt(log(2/b) / (2*obj.gamma_i_u^(-1))) + c;
                otherwise
                    error('Norm not supported')
            end

            % Ensure radius is at least as large as for varying transitions
            if r < obj.r_i_v
                r = obj.r_i_v;
            end
        end


        function [P_hat,r] = update(obj, samples)
        % Update ambiguity set and estimated probabilities using new samples

            n_samples = length(samples);
            P_hat = zeros(1,length(obj.P_i_hat));
            for k = 1:n_samples
                % Count occurrences of unknown transitions
                if any(obj.I_i_u==samples(k))
                    P_hat(samples(k)) = P_hat(samples(k)) + 1;
                end
            end

            n_samples_u = sum(P_hat); % total number of unknown transition samples

            if n_samples_u == 0
                % No new samples: keep previous estimate and radius
                P_hat = obj.P_i_hat;
                r = obj.r_i;
                return
            else
                % Normalize to get empirical estimation of probabilities for unknowns
                P_hat = P_hat/n_samples_u;
                obj.p_ij_cond_u(obj.I_i_u) = P_hat(obj.I_i_u);
                % Scale by estimated total probability mass for unknowns
                P_hat = P_hat * obj.p_i_u_hat;
            end

            % Set known and varying transitions to their previous values
            P_hat([obj.I_i_k,obj.I_i_v]) = obj.P_i_hat([obj.I_i_k,obj.I_i_v]);

            if obj.u_i > 0 
                % Update inverse sample size and ambiguity radius
                obj.gamma_i_u = n_samples_u^(-1);
                r = get_radius(obj);
            else
                r = 0;
            end

            obj.P_i_hat = P_hat;
            obj.r_i = r;
        end
    end
end

