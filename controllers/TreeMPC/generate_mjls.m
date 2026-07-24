% GENERATE_MJLS computes the switched system matrices of a real-time control system
% subject to deadline misses
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_tilde,B_tilde] = generate_mjls(A_c,B_c,strategy_overrun,strategy_actuator,period,nodes)
% Computes the switched discrete-time system matrices from the continuous-time matrices 
% depending on the overrun and actuator strategies, the sampling time and possible modes.
%
% Inputs
%   A_c, B_c            : continuous time state-space matrices
%   strategy_overrun    : overrun strategy ('kill' or 'skip')
%   strategy_actuator   : actuator strategy ('hold' or 'zero')
%   period              : sampling period
%   nodes               : list of nodes (char arrays of hit/miss sequence,
%                         nodes correspond to Markov modes)
%
% Outputs
%   A_tilde             : cell array of system matrices of extended system
%   B_tilde             : cell array of input matrices of extended system

    nu = size(B_c,2);
    nx = size(A_c,1);

    ss_c = ss(A_c,B_c,eye(nx),zeros(nx,nu));
    ss_d = c2d(ss_c,period);
    A = ss_d.A;
    B = ss_d.B;
    
    % define matrices for hit and miss dynamics
    actuator_mode = strcmp(strategy_actuator,'hold');
    switch strategy_overrun
        case 'kill'
            A_tilde_hit = [A, B;
                           zeros(nu,nx+nu)];
            A_tilde_miss = [A, B;
                            zeros(nu,nx), actuator_mode*eye(nu)];
            B_tilde_hit = [zeros(nx,nu); 
                           eye(nu)];
            B_tilde_miss = zeros(nx+nu,nu);
        case 'skip'
            A_tilde_hit = [A, zeros(nx,nu), B;
                           zeros(nu,nx+nu), eye(nu);
                           zeros(nu,nx+2*nu)];
            A_tilde_miss = [A, B, zeros(nx,nu);
                            zeros(nu,nx), actuator_mode*eye(nu), zeros(nu,nu);
                            zeros(nu,nx+nu), eye(nu)];
            B_tilde_hit = [zeros(nx+nu,nu); 
                           eye(nu)];
            B_tilde_miss = zeros(nx+2*nu,nu);
        otherwise
            disp('not implemented')
    end

    % assign each node (Markov mode) the corresponding switched matrices
    % with nodes ending in 1 corresponding to a hit mode
    for i=1:numel(nodes)
        if endsWith(nodes{i},'1')
            A_tilde{i} = A_tilde_hit;
            B_tilde{i} = B_tilde_hit;
        else
            A_tilde{i} = A_tilde_miss;
            B_tilde{i} = B_tilde_miss;
        end
    end
end

