% MARKOV_DR_CTRL_COMMON computes a distributionally robust non-switching 
% state feedback controller for a MJLS.
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [K,sol] = markov_dr_ctrl_common(ambsets,A,B,Q,R,obj_func)
% Implements the LMI based synthesis of a distributionally robust
% non-switching state feedback controller for a MJLS.
%
% Inputs
%   ambsets     : ambiguity sets for each row of the transition matrix
%   A, B        : cell arrays of switched system dynamics 
%   Q, R        : cost matrices from Lyapunov-like function
%   obj_func    : objective function for optimization ('none', 'trace', 'logdet')
%
% Outputs
%   K           : controller gain
%   sol         : struct with information about optimization problem
    
    % initialize estimated transition probability matrix
    P_hat = zeros(length(ambsets{1}.P_i_hat));

    nr = size(P_hat,1);  
    nx = size(A{1},1);
    nu = size(B{1},2);

    % initialize optimization variables
    E = sdpvar(nx,nx,'symmetric');
    Y = sdpvar(nu,nx);
    constr = [E >= 0];

    for i=1:nr
        for j = 1:nr
            if ~ismember(j,ambsets{i}.I_i_k)
                St{i,j} = sdpvar(nx,nx);
                constr = [constr, St{i,j} >= 0];
            else
                St{i,j} = [];
            end
        end
    end

    for i = 1:nr    
        % get parameters of ambiguity set of mode i
        n_i_uv = ambsets{i}.u_i + ambsets{i}.v_i;
        n_i_k = ambsets{i}.k_i;
        p_i_uv = 1-ambsets{i}.p_i_k;
        P_hat(i,:) = ambsets{i}.P_i_hat;
        I_i_k = ambsets{i}.I_i_k;
        I_i_uv = setdiff(1:nr,I_i_k);
        
        % define LMI blocks with known transition probabilities
        mat_k = [];
        Et = [];
        for k = 1:n_i_k
            if P_hat(i,I_i_k(k)) > 1e-9
                mat_k = [mat_k, sqrt(P_hat(i,I_i_k(k))) * (A{I_i_k(k)}*E+B{I_i_k(k)}*Y)'];
                Et = blkdiag(Et,E);
            end
        end
    
        % LMI formulation depends on the norm l_p used to define the 
        % ambiguity set
        switch ambsets{i}.l_p
            case 1 

                St_sum = 0;
                for l = 1:n_i_uv
                    St_sum = St_sum + P_hat(i,I_i_uv(l))*St{i,I_i_uv(l)};
                end

                for j = 1:n_i_uv
                    % closed-loop dynamics of mode I_i_uv(j)
                    AE_BY = A{I_i_uv(j)}*E+B{I_i_uv(j)}*Y;
                    for h = 1:n_i_uv
                        lmi_pm = { ambsets{i}.r_i*St{i,I_i_uv(h)}, -ambsets{i}.r_i*St{i,I_i_uv(h)}};
                        for idx_pm = 1:2
        
                            LMI1 = [E + p_i_uv*St{i,I_i_uv(j)} + lmi_pm{idx_pm} - St_sum,    E*Q^(1/2),            Y'*R^(1/2),           sqrt(p_i_uv)*AE_BY',  mat_k;
                                    Q^(1/2)*E,                                      	     eye(nx),              zeros(nx,nu),         zeros(nx),            zeros(nx,n_i_k*nx);
                                    R^(1/2)*Y,                                               zeros(nu,nx),         eye(nu),              zeros(nu,nx),         zeros(nu,n_i_k*nx);
                                    sqrt(p_i_uv)*AE_BY,                                      zeros(nx),            zeros(nx,nu),         E,                    zeros(nx,n_i_k*nx);
                                    mat_k',                                                  zeros(n_i_k*nx,nx),   zeros(n_i_k*nx,nu),   zeros(n_i_k*nx,nx),   Et];

                            constr = [constr, LMI1 >= 0];
                        end
                    end
                end

            case inf

                St_sum = 0;
                for l = 1:n_i_uv
                    St_sum = St_sum + (ambsets{i}.r_i + P_hat(i,I_i_uv(l)))*St{i,I_i_uv(l)};
                end

                for j = 1:n_i_uv
                    % closed-loop dynamics of mode I_i_uv(j)
                    AE_BY = A{I_i_uv(j)}*E+B{I_i_uv(j)}*Y;

                    LMI1 = [E + p_i_uv*St{i,I_i_uv(j)} - St_sum,    E*Q^(1/2),                Y'*R^(1/2),               sqrt(p_i_uv)*AE_BY',        mat_k;
                            Q^(1/2)*E,                              eye(nx),                  zeros(nx,nu),             zeros(nx),                  zeros(nx,size(mat_k,2));
                            R^(1/2)*Y,                              zeros(nu,nx),             eye(nu),                  zeros(nu,nx),               zeros(nu,size(mat_k,2));
                            sqrt(p_i_uv)*AE_BY,                     zeros(nx),                zeros(nx,nu),             E,                          zeros(nx,size(mat_k,2));
                            mat_k',                                 zeros(size(mat_k,2),nx),  zeros(size(mat_k,2),nu),  zeros(size(mat_k,2),nx),    Et];

                    constr = [constr, LMI1 >= 0];
                end
        end

        if n_i_uv == 0

            LMI1 = [E,          E*Q^(1/2),                Y'*R^(1/2),               mat_k;
                    Q^(1/2)*E,  eye(nx),                  zeros(nx,nu),             zeros(nx,size(mat_k,2));
                    R^(1/2)*Y,  zeros(nu,nx),             eye(nu),                  zeros(nu,size(mat_k,2));
                    mat_k',     zeros(size(mat_k,2),nx),  zeros(size(mat_k,2),nu),  Et];

            constr = [constr, LMI1 >= 0];
        end
    end

    % define objective function
    switch obj_func
        case 'none'
            obj = 1;
        case 'logdet'
            obj = -logdet(E);
        case 'trace'
            obj = -trace(E);
    end
    
    % solve optimization problem
    ops = sdpsettings('solver','mosek','verbose',0);
    sol = optimize(constr, obj, ops);
    
    % get optimal values
    E = value(E);
    P = inv(E);
    Y = value(Y);
    K = Y*P;
end

