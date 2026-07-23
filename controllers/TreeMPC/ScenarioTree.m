% SCENARIOTREE spans a scenario tree based on a Markov transition matrix
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

classdef ScenarioTree < handle
% Spans a scenario tree based on a Markov transition matrix.
%
% Properties:
%   nodes               : tree nodes
%   leaf_nodes          : leaf nodes (nodes without child nodes)
%   tpm                 : transition probability matrix
%   prediction_horizon  : number of stages of tree = prediction horizon + 1
%   sample_paths        : all paths from root to leaf nodes
%   num_sample_paths    : number of paths = number of leaf nodes
%
% Methods
%   ScenarioTree        : Constructor
%   add_node            : adds a node object to the tree
%   get_nodes_at_stage  : returns all nodes at a given stage
%   set_children        : defines the child nodes of a given node
%   compute_paths       : computes all paths from root node to leaf nodes
%   trace_sample_paths  : computes a path for a given leaf node
    
    properties
        nodes               % tree nodes
        leaf_nodes          % leaf nodes
        tpm                 % transition probability matrix
        prediction_horizon  % prediction horizon 
        sample_paths        % all sample paths through the scenario tree
        num_sample_paths    % number of sample paths
    end
    
    methods
        function obj = ScenarioTree(tpm,prediction_horizon,initial_mode)
        % Constructor
        %
        % Inputs
        %   tpm                 : transition probability matrix
        %   prediction_horizon  : prediction horizon
        %   initial_mode        : Markov mode of root node

            obj.tpm = tpm;
            obj.prediction_horizon = prediction_horizon;
            obj.nodes = [];
            obj.sample_paths = [];

            % construct the scenario tree starting at root node
            stage = 1;
            ancestor = -1;
            state.mode = initial_mode;
            state.probability = 1;
            add_node(obj, stage, ancestor, state, []);     % root node
            
            % construct stage-wise
            for k = 2:prediction_horizon + 1
                % get all nodes at previous stage
                ancestor_list = get_nodes_at_stage(obj, k-1);

                % construct for all nodes at stage k-1 all child nodes at
                % stage k
                for j = 1 : length(ancestor_list)
                    ancestor_j = ancestor_list(j);
                    prob_j = ancestor_j.state.probability;
                    mode_j = ancestor_j.state.mode;

                    % potential child node i
                    for mode_i = 1:size(obj.tpm,1)
                        state.mode = mode_i;
                        state.probability = prob_j * obj.tpm(mode_j,mode_i);
                        if state.probability > 0
                            add_node(obj,k,ancestor_list(j).id,state,[]);
                        end
                    end
                end
            end
            
            % assign the child indices for all nodes
            set_children(obj);
            % determine leaf nodes as the tree nodes at the last stage
            obj.leaf_nodes = get_nodes_at_stage(obj,prediction_horizon+1);
            % compute all paths through the tree
            obj.compute_paths();
        end
        
        function add_node(obj, stage, ancestor, state, child_idx)
        % create and add a node to the tree
            num_nodes = length(obj.nodes)+1;
            node_new = TreeNode(num_nodes,stage,ancestor,state,child_idx);
            obj.nodes = [obj.nodes node_new];
        end

        function stage_nodes = get_nodes_at_stage(obj, stage)
        % returns a list of all nodes at the given stage
            all_nodes = obj.nodes;
            stage_nodes = [];
            for i = 1:length(all_nodes)
                if all_nodes(i).stage == stage
                    stage_nodes = [stage_nodes all_nodes(i)];
                end
            end
        end

        function set_children(obj)
        % assign to each node their child node indices
            all_nodes = obj.nodes;
            % go through tree stage-wise
            for k = 2:obj.prediction_horizon+1
                anc_nodes   = get_nodes_at_stage(obj, k-1);
                stage_nodes = get_nodes_at_stage(obj, k);
                % for all nodes of previous stage
                for i = 1:length(anc_nodes)
                    child_list = [];
                    for j = 1 : length(stage_nodes)
                        if anc_nodes(i).id == stage_nodes(j).ancestor
                            child_list = [child_list, stage_nodes(j).id];
                        end
                    end
                    all_nodes(anc_nodes(i).id).child_idx = child_list;
                end
            end
            obj.nodes = all_nodes;
        end

        function compute_paths(obj)
        % extract all sample paths from root to leaf nodes
            for i = 1:length(obj.leaf_nodes)
                id_vec      = obj.leaf_nodes(i).id;
                mode_vec    = obj.leaf_nodes(i).state.mode;
                prob_vec    = obj.leaf_nodes(i).state.probability;
                % get path ending at current leaf node
                [id_vec, mode_vec, prob_vec] = trace_sample_paths(obj, obj.leaf_nodes(i).id, id_vec, mode_vec, prob_vec);
                obj.sample_paths = [obj.sample_paths, struct('modes', mode_vec, 'prob', prob_vec, 'idcs', id_vec)];
            end
            % store total number of sample paths
            obj.num_sample_paths = length(obj.sample_paths);
        end

        function [id_vec, mode_vec, prob_vec] = trace_sample_paths(obj, id, id_vec, mode_vec, prob_vec)
        % recursively determine a path ending at the given leaf node
            id_anc = obj.nodes(id).ancestor;
            % if root node, then break the recursion
            if id_anc == -1
                return
            end 

            % concatenate id, mode and probability paths
            id_vec = [id_anc, id_vec];
            mode_vec = [obj.nodes(id_anc).state.mode,  mode_vec];
            prob_vec = [obj.nodes(id_anc).state.probability, prob_vec];

            % recursively call the function until root node is reached
            [id_vec, mode_vec, prob_vec] = trace_sample_paths(obj, id_anc, id_vec, mode_vec, prob_vec);
        end
    end
end

