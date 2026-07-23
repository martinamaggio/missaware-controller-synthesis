% MARKOV_MPC_SOFT compiles the optimal control problem for a soft-constrained 
% model predictive control approach for a MJLS based on a scenario tree
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function controller = markov_mpc_soft(mpc,A,B,tree,use_node_slack)
% Compiles the optimal control problem for a soft-constrained 
% model predictive control approach for Markov jump linear systems
% based on a scenario tree. All state constraints are treated as soft
% constraints, input constraints as hard constraints.
%
% Inputs
%   mpc                     : struct containing the mpc parameters
%                             (prediction horizon: N,
%                              mode-dependent terminal set matrices: P,
%                              state objective function matrix: Q,
%                              input objective function matrix: R,
%                              slack penalty function matrix: S,
%                              hard input constraints with L*u<=lu:
%                              input_constraint.L, input_constraint.lu,
%                              soft state constraints with H*x<=hx:
%                              state_constraint.H, state_constraint.hx)
%   A, B                    : cell arrays of switched system dynamics
%   tree                    : ScenarioTree object
%   use_node_slack          : use slack variable per node or per stage
%
% Outputs
%   controller              : optimal control problem (Yalmip optimizer object)

    % get dimensions of problem
    nu = size(B{1}, 2);
    nx = size(A{1}, 1);
    num_leaves = length(tree.leaf_nodes);
    num_nodes = length(tree.nodes);
    
    % define optimization variables x, u, e (slack variables)
    x = sdpvar(nx*ones(1,num_nodes),ones(1,num_nodes));
    u = sdpvar(nu*ones(1,num_nodes-num_leaves),ones(1,num_nodes-num_leaves));
    
    % non terminal slack variables can be defined per node or per stage of
    % the scenario tree
    if use_node_slack
        e = sdpvar(length(mpc.state_constraint.hx)*ones(1,num_nodes-num_leaves),ones(1,num_nodes-num_leaves));
    else
        e = sdpvar(length(mpc.state_constraint.hx)*ones(1,mpc.N),ones(1,mpc.N));
    end

    for r=1:numel(mpc.P)
        mpc.L{r} = chol(mpc.P{r}, 'lower');
    end
    
    % initialize constraints and objective function
%     constraints = [norm(u{1},2) <= 100]; % force problem to be QCQP
    constraints = [];
    objective = 0;
    
    % slack penalty function is implemented as linear function
    mpc.S = mpc.S(:)';
    if size(mpc.S,1) > 1
        error('incorrect dimension of S')
    end

    % initialize terminal slack and associated variables
    c = component_termset_constraint(mpc.P,mpc.state_constraint.H);
    nhx = length(mpc.state_constraint.hx);
    if ~is_symmetric_constraints(mpc.state_constraint.H,mpc.state_constraint.hx)
        use_full_terminal_slack = true;
        es_full = sdpvar(nhx,1);
        constraints = [constraints, es_full >= zeros(nhx,1)];
    else
        % if the state constraints are symmetric, only one terminal slack 
        % variable per pair of symmetric constraints is defined due to
        % symmetry of terminal set
        use_full_terminal_slack = false;
        es = sdpvar(nhx/2,1);
        constraints = [constraints, es >= zeros(nhx/2,1)];
        for i = 1:nhx/2
            es_full(2*i-1:2*i,1) = es(i);
        end
        c = c(1:2:end,:);
    end

    % set up constraints along sample paths of scenario tree
    for i = 1 : tree.num_sample_paths 
        idx = tree.sample_paths(i).idcs;   % indices of path nodes
        mode = tree.sample_paths(i).modes; % modes of path nodes
        prob_scenario = tree.sample_paths(i).prob(end); % total probability of sample path

        for k = 1:length(idx) - 1 % for all nodes along the sample path
            % switched system dynamics
            constraints = [constraints, x{idx(k+1)} == A{mode(k)}*x{idx(k)} + B{mode(k)}*u{idx(k)}]; 
            % hard input constraints
            for ll = 1 : length(mpc.input_constraint.lu)
                constraints = [constraints, mpc.input_constraint.L(ll,:)*u{idx(k)} <= mpc.input_constraint.lu(ll)];
            end
            % soft state constraints with stage- or node-specific slack
            % variables
            if use_node_slack
                ei = e{idx(k)};
            else
                ei = e{k};
            end
            for ll = 1 : length(mpc.state_constraint.hx)
                constraints = [constraints, mpc.state_constraint.H(ll,:)*x{idx(k)} <= mpc.state_constraint.hx(ll)+ei(ll)+es_full(ll)];
            end
            constraints = [constraints, ei >= zeros(length(mpc.state_constraint.hx),1)];
            % objective function
            objective = objective + prob_scenario * (x{idx(k)}'* mpc.Q*x{idx(k)} + u{idx(k)}'*mpc.R*u{idx(k)}...
                                                      + mpc.S*(ei+es_full) );
        end

        % terminal cost function and constraints
        objective = objective + prob_scenario * (x{idx(end)}' * mpc.P{mode(end)} * x{idx(end)} + mpc.S * es_full);
        constraints = [constraints, norm(x{idx(end)}'*mpc.L{mode(end)},2) <= 1]; % force problem to be recognized as QCQP
        if use_full_terminal_slack
            constraints = [constraints, c*x{idx(end)}'*mpc.P{mode(end)}*x{idx(end)} <= mpc.state_constraint.hx+es_full];
        else
            constraints = [constraints, c*x{idx(end)}'*mpc.P{mode(end)}*x{idx(end)} <= mpc.state_constraint.hx(1:2:end)+es];
        end
    end

    % initial state
    x0 = sdpvar(nx,1);
    constraints = [constraints, x{1} == x0];

    % define optimizer object
    mpc_input = {x0};
    mpc_output = {[u{:}], [x{:}], [e{:}], es_full, objective};
    options = sdpsettings('solver','mosek','verbose',0);
    controller = optimizer(constraints,objective,options,mpc_input,mpc_output);
end


function res = is_symmetric_constraints(H,hx)
% determine if the state constraints are all symmetric
    res = true;
    for i=1:2:length(hx)
        if hx(i)~=hx(i+1) || any(-H(i,:)~=H(i+1,:))
            res = false;
            return
        end
    end
end

function c = component_termset_constraint(terminal_set,constraints_matrix)
% compute variable used to define terminal slack values depending on
% maximum soft constraint violation with mode-dependent terminal set
    nh = size(constraints_matrix,1);
    nr = numel(terminal_set);
    c_temp = zeros(nh,nr);
    for r = 1:nr
        Br = terminal_set{r}^(-1/2);
        for i = 1:nh
            c_temp(i,r) = norm(Br*constraints_matrix(i,:)');
        end
    end
    c = max(c_temp,[],2);
end