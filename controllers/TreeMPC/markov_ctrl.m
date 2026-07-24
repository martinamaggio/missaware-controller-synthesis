% MARKOV_CTRL computes a switching state feedback controller and mode-dependent invariant set for a MJLS
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [P,K,sol] = markov_ctrl(A,B,Q,R,tpm,H,hx,L,lu,obj_func)
% Implements the LMI based synthesis of a switching state feedback controller 
% and mode-dependent invariant set inside the given constraints for a MJLS.
%
% Inputs
%   A, B        : cell arrays of switched system dynamics 
%   Q, R        : cost matrices from Lyapunov-like function
%   tpm         : transition probability matrix
%   H, hx       : state constraints (H*x <= hx)
%   L, lu       : input constraints (L*u <= lu)
%   obj_func    : objective function for optimization ('none', 'trace', 'logdet')
%
% Outputs
%   P           : cell array of terminal set matrices
%                 with X_i = {x | x'*P{i}*x <= 1}
%   K           : cell array of controller gains
%   sol         : struct with information about optimization problem

    n_r = size(tpm,2);    
    n_x = size(A{1},1);
    n_u = size(B{1},2);
    
    % initialize optimization variables
    E_bar = [];
    for i=1:n_r
        Y{i} = sdpvar(n_u,n_x);
        E{i} = sdpvar(n_x,n_x,'symmetric');
        E_bar = blkdiag(E_bar,E{i});
    end

    % initialize constraints of optimization problem
    constr = [E_bar >= 0];

    for i = 1:n_r    
        f_i = zeros(1,n_r);
        for j=1:n_r
            if tpm(i,j) >= 1e-9
                f_i(j) = sqrt(tpm(i,j));
            end
        end
        F_i = kron(f_i, eye(n_x));

        % closed-loop dynamics of mode i
        AE_BY = A{i}*E{i} + B{i}*Y{i};

        % stochastic decrease constraint
        LMI1 = [E{i},               AE_BY'*F_i,             E{i}*Q^(1/2),       Y{i}'*R^(1/2);
                F_i'*AE_BY,         E_bar,                  zeros(n_r*n_x,n_x), zeros(n_r*n_x,n_u);
                Q^(1/2)*E{i},       zeros(n_x,n_r*n_x),     eye(n_x),           zeros(n_x,n_u);
                R^(1/2)*Y{i},       zeros(n_u,n_r*n_x),     zeros(n_u,n_x),     eye(n_u)];

        constr = [constr, LMI1 >= 0];

        % uniform asymptotic stability constraint for all possible transitions
        for j=1:n_r
            if tpm(i,j) >= 1e-9
                LMI2 = [E{i},       AE_BY';
                        AE_BY,      E{j}];

                constr = [constr, LMI2 >= 0];
            end
        end

        % state constraints
        for j = 1 : size(H,1)
            LMI3 = [1,                      H(j,:)/hx(j)*E{i};
                    E{i}'*H(j,:)'/hx(j),    E{i}];
            constr = [constr, LMI3 >= 0];
        end
    
        % input constraints
        for j = 1 : size(L,1)
            LMI3 = [1,                      L(j,:)/lu(j)*Y{i};
                    Y{i}'*L(j,:)'/lu(j),    E{i}];
            constr = [constr, LMI3 >= 0];
        end

    end

    % define objective function
    switch obj_func
        case 'none'
            obj = 1;
        case 'logdet'
            obj = -logdet(E_bar);
        case 'trace'
            obj = -trace(E_bar);
    end
   
    ops = sdpsettings('solver','mosek','verbose',0);
    sol=optimize(constr, obj, ops);
    
    for i=1:n_r
        E{i} = value(E{i});
        P{i} = inv(E{i});
        K{i} = value(Y{i})*P{i};
        if sum(abs(K{i}),'all') < 1e-9
            K{i} = zeros(size(K{i}));
        end
    end
end

