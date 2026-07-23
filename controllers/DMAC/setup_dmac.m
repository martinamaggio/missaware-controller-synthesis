% SETUP_DMAC implements the DMAC state feedback controller
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [ctrl_func, z0] = setup_dmac(psys,prt)
% Implements the DMAC state feedback controller using a stochastic anyMiss(m,k)
% constraint to compute probabilistic hold and delay values.
%
% Inputs
%   psys    : struct containing the system and simulation parameters
%   prt     : struct of the real-time setting parameters
%
% Outputs
%   ctrl_func   : control function wrapper
%   z0          : initial controller state

    % extract the needed system and real-time parameters
    plant_name = psys.plant_name;               % plant name ('furuta' or 'motor' implemented)
    period = psys.period;                       % sampling period
    m = prt.m; k = prt.w;                       % parameters of anyMiss constraint
    sequence = prt.sample_sequence;             % sample sequence for estimation of transition probability matrix
    overrun_strategy = prt.overrun_strategy;    % overrun strategy ('kill' or 'skip')
    actuator_strategy = prt.actuator_strategy;  % actuator strategy ('hold' or 'zero')

    % compute delay and hold matrices using a stochastic anyMiss(m,k) graph
    nodes = stochastic_anymiss_graph(m,k,[]);
    tpm = estimate_probabilities(nodes,sequence);
    possible_node_idcs = find(sum(tpm,1)>0);
    tpm = tpm(possible_node_idcs,possible_node_idcs);
    nodes = nodes(possible_node_idcs);
    [delay_mat,hold_mat] = stoch_anymiss_graph_to_hold_delay(nodes,tpm,overrun_strategy,actuator_strategy);
    delay_mat = [delay_mat(:,1), zeros(size(delay_mat,1),1), delay_mat(:,2)];
    
    % nominal system dynamics 
    [Ac,Bc] = feval(['dmac_param_',plant_name]);
    nx = size(Ac,1);
    nu = size(Bc,2);
    Qc = eye(nx+nu);
    
    % compute DMAC
    ctrl = robust_lqctrl(Ac,Bc,Qc,period,hold_mat,delay_mat,overrun_strategy,actuator_strategy);
    
    % function wrapper for dynamic controller
    function [u,z] = ctrl_wrapper(x,z,u,sequence,~)
        u = ctrl.C * z + ctrl.D * x;
        z = ctrl.A * z + ctrl.B * x;
    end

    ctrl_func = @ctrl_wrapper;
    z0 = zeros(size(ctrl.C,2),1);
end


function p_miss = get_miss_probabilities(sequence)
% returns a vector of consecutive miss probabilities

    % extract the lengths of all sequences of consecutive misses
    seq_changes = diff([1,sequence,1]);
    starts = find(seq_changes == -1);  % miss sequences start when the sequence changes from 1 to 0
    ends = find(seq_changes == 1);   % miss sequences end when the sequence changes from 0 to 1
    lengths = ends-starts;

    n_hit_starts = sum(sequence(1:end-1))+1;
    max_length = max(lengths);

    p_miss = zeros(1,max_length+1);
    p_miss(1) = length(starts)/n_hit_starts;

    num_longer_lengths = sum(lengths>=[1:max_length+1]',2);
    for i=2:max(lengths)+1
        p_miss(i) = num_longer_lengths(i)/num_longer_lengths(i-1);
    end
end
