% TREENODE represents a node of a scenario tree
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

classdef TreeNode
% Represents a node of a scenario tree.
%
% Properties
%   ancestor    : ancestor/predecessor node
%   stage       : scenario tree stage of this node
%   id          : number of node
%   state       : state of node (Markov mode, node probability)
%   child_idx   : indices of child/successor nodes
%
% Methods
%   TreeNode    : Constructor
    
    properties
        ancestor
        stage
        id
        state
        child_idx
    end
    
    methods
        function obj = TreeNode(id, stage, ancestor, state, child_idx)
            obj.id = id;
            obj.ancestor = ancestor;
            obj.stage = stage;
            obj.state = state;
            obj.child_idx = child_idx;
        end
    end
end

