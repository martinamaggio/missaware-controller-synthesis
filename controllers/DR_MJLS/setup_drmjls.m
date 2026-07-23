% SETUP_DRMJLS implements the DR-MJLS state feedback controller for any weakly-hard constraint
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [ctrl_func, z0] = setup_drmjls(psys,prt,switching)
% Implements the distributionally robust DR-MJLS (switching or non-switching) 
% state feedback controller for any weakly-hard constraint.
%
% Inputs
%   psys                : struct containing the system and simulation parameters
%   prt                 : struct of the real-time setting parameters
%   switching           : true or false
%
% Outputs
%   ctrl_func           : function handle for the controller
%   z0                  : initial controller state (stateless: z0=[])

    % extract the needed system and real-time parameters
    plant_name = psys.plant_name;               % plant name ('furuta' or 'motor' implemented)
    period = psys.period;                       % sampling period
    wh_constraint = prt.wh_constraint;          % type of weakly-hard constraint ('any_miss', 'any_hit', 'row_miss', 'row_hit')
    m = prt.m; k = prt.w; h = prt.h;            % parameters of weakly-hard constraint
    training_sequence = prt.sample_sequence;    % sample sequence for estimation of transition probability matrix
    strategy_overrun = prt.overrun_strategy;    % overrun strategy ('kill' or 'skip')
    strategy_actuator = prt.actuator_strategy;  % actuator strategy ('hold' or 'zero')

    % plant specific parameters
    [A_c,B_c,Q,R] = feval(['drmjls_param_',plant_name],strategy_overrun);

    % model the MJLS for the selected weakly-hard constraint 
    switch wh_constraint
        case 'any_miss'
            [nodes,tpm] = stochastic_anymiss_graph(m,k,[]);
        case 'any_hit'
            [nodes,tpm] = stochastic_anyhit_graph(h,k,[]);
        case 'row_miss'
            [nodes,tpm] = stochastic_rowmiss_graph(m,[]);
        case 'row_hit'
            [nodes,tpm] = stochastic_rowhit_graph(h,k,[]);
        otherwise
            error('WH constraint not valid')
    end    
    [A_tilde,B_tilde] = generate_mjls(A_c,B_c,strategy_overrun,strategy_actuator,period,nodes);

    l_p = inf;   % norm used to define ambiguity sets
    beta = 0.05; % 1-beta: confidence level of 

    %% Ambiguity sets
    % define structure and initialize an ambiguity set for each row of the
    % transition probability matrix
    % (for this example it is assumed that any transitions probabilities
    % that are 0 or 1 are known and the others are static but unknown and
    % will be estimated from data)
    nr = size(tpm,1);
    I_k = cell(1,nr); % cell array of indices j of known p_ij per row i
    p_k = cell(1,nr); % cell array of values of known p_ij
    I_u = cell(1,nr); % cell array of indices of unknown (estimated from data) p_ij
    for i=1:nr
        % define known probabilities
        I_k{i} = find(tpm(i,:) == 0 | tpm(i,:) == 1);
        p_k{i} = tpm(i,I_k{i});
        % define unknown static probabilities
        I_u{i} = setdiff(1:nr,I_k{i});
    end
    % create a Markov chain learner object
    mcl = MarkovChainLearner(nr,beta,I_k,p_k,cell(nr),cell(nr),l_p);

    % translate training sequence into mode sequence and learn transition matrix 
    modes = [];
    for i=1:length(training_sequence)
        sequence_win = training_sequence(max(1,i-k+1):i);
        sequence_win = char(strjoin(string(sequence_win),''));
        mode_win = find_node(nodes,sequence_win);
        if ~isempty(mode_win)
            modes = [modes, mode_win];
        end
    end
    mcl.update_ambsets(modes);


    %% Controller design
    % design a switching or non-switching static state feedback controller    
    yalmip('clear')
    switch switching
        case true
            [K,sol] = markov_dr_ctrl_switching(mcl.ambiguity_set,A_tilde,B_tilde,Q,R,'trace');
        case false
            [K{1},sol] = markov_dr_ctrl_common(mcl.ambiguity_set,A_tilde,B_tilde,Q,R,'trace');
        otherwise
            error('Value of switching variable is invalid')
    end
    if sol.problem
        error('No valid optimization result found')
    end

    function [u,z] = ctrl_wrapper(x,z,u,sequence,~)
    % Control wrapper to use in simulation script
        % kill: xt = [x; u]
        % skip next: xt = [x; z_1; z_2]
        %   controller only called after a hit -->
        %       z_1(k): not needed
        %       z_2(k): set to u0
        
        sequence = char(strjoin(string(sequence),''));
        if strategy_overrun == "kill"
            xt = [x; u];
            % sequence is known up to the current hit
            sequence_known = sequence;
        elseif strategy_overrun == "skip"
            xt = [x; zeros(size(u)); u];
            % control job started after last hit, sequence in between last
            % and current hit unknown
            idx_hit = find(sequence=='1');
            sequence_known = sequence(1:idx_hit(end-1));
        end

        if switching
            % identify the node index of the known sequence to choose the
            % appropriate switched controller gains
            i_node = find_node(nodes,sequence_known);
            if isempty(i_node)
                i_node = 1;
            end
        else
            i_node = 1;
        end

        u = K{i_node}*xt;
    end

    ctrl_func = @ctrl_wrapper;
    z0 = [];
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