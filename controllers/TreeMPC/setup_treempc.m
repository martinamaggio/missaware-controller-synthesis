% SETUP_TREEMPC implements the soft-constrained scenario-based MPC for any weakly-hard constraint.
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [ctrl_func, z0] = setup_treempc(psys,prt)
% Implements the soft-constrained scenario-based MPC for any weakly-hard constraint.
%
% Inputs
%   psys                : struct containing the system and simulation parameters
%   prt                 : struct of the real-time setting parameters
%
% Outputs
%   ctrl_func          : function handle for the MPC controller
%   z0                 : initial controller state (stateless: z0=[])

    % extract the needed system and real-time parameters
    plant_name = psys.plant_name;               % plant name ('furuta' or 'motor' implemented)
    period = psys.period;                       % sampling period
    wh_constraint = prt.wh_constraint;          % type of weakly-hard constraint ('any_miss', 'any_hit', 'row_miss', 'row_hit')
    m = prt.m; k = prt.w; h = prt.h;            % parameters of weakly-hard constraint
    sequence = prt.sample_sequence;             % sample sequence for estimation of transition probability matrix
    strategy_overrun = prt.overrun_strategy;    % overrun strategy ('kill' or 'skip')
    strategy_actuator = prt.actuator_strategy;  % actuator strategy ('hold' or 'zero')
    
    % plant specific parameters
    [A_c,B_c,mpc] = feval(['treempc_param_',plant_name],strategy_overrun);

    % Model the MJLS for the selected weakly-hard constraint 
    switch wh_constraint
        case 'any_miss'
            nodes = stochastic_anymiss_graph(m,k,[]);
        case 'any_hit'
            nodes = stochastic_anyhit_graph(h,k,[]);
        case 'row_miss'
            nodes = stochastic_rowmiss_graph(m,[]);
        case 'row_hit'
            nodes = stochastic_rowhit_graph(h,k,[]);
        otherwise
            error('WH constraint not valid')
    end

    tpm = estimate_probabilities(nodes,sequence);
    possible_node_idcs = find(sum(tpm,1)>0);
    tpm = tpm(possible_node_idcs,possible_node_idcs);
    nodes = nodes(possible_node_idcs);
    [A_tilde,B_tilde] = generate_mjls(A_c,B_c,strategy_overrun,strategy_actuator,period,nodes);

    %% Controller design    
    % Design terminal sets
    yalmip('clear')
    [mpc.P,mpc.K,sol] = markov_ctrl(A_tilde,B_tilde,mpc.Q,mpc.R,tpm,[],[],...
                             mpc.input_constraint.L,mpc.input_constraint.lu,'logdet');
    if sol.problem
        error('No valid optimization result for terminal sets')
    end
    yalmip('clear')

    % initialize scenario tree and corresponding optimal control problem
    % for the relevant modes (B =/= 0)
    tree = cell(size(tpm,1),1);
    ctrl = cell(size(tpm,1),1);
    for i=1:size(tpm,1)
        if ~isequal(B_tilde{i}, zeros(size(B_tilde{i})))
            % define scenario tree and optimal control problem
            tree{i} = ScenarioTree(tpm,mpc.N,i);
            ctrl{i} = markov_mpc_soft(mpc,A_tilde,B_tilde,tree{i},true);
        end
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
        
        % identify the node index of the known sequence to choose the
        % compiled optimal control problem based on the scenario tree 
        % starting at the appropriate initial mode
        i_node = find_node(nodes,sequence_known);
        if isempty(i_node)
            i_node = 1;
        end
        
        [output, problem, err_msg] = ctrl{i_node}({xt});
        if ~isempty(output{1}) && ~any(isnan(output{1}),'all')
            u = output{1}(:,1);
        else
            disp(err_msg)
        end
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
