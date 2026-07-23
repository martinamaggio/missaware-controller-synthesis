% CTRL_MOTOR_STATE designs a linear dynamic state feedback controller for an electric motor 
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [F,G,H,K,C_ext] = ctrl_motor_state(sampling_period,~)
% Designs a linear dynamic state feedback controller for the linear model 
% of the electric motor with one-step-delayed actuation.
%
% Inputs
%   sampling_period : sample time
% 
% Outputs
%   F,G,H,K         : matrices of the dynamic controller:
%                       z[k+1] = F z[k] + G y[k]
%                       u[k+1] = H z[k] + K y[k]
%   C_ext           : extended output matrix for one-step-delayed actuation

    [Ac,Bc] = motor_matrices();
    nx = size(Ac,1);
    nu = size(Bc,2);

    % constant discrete-time matrices of extended system
    % where x_extended(k) = [x(k); u(k-1)] models a one-step delay
    B = [zeros(nx,nu); eye(nu)];

    nxt = nx+nu;
    Q_ctrl = eye(nxt);
    R_ctrl = eye(nu);

    sysc = ss(Ac,Bc,eye(nx),zeros(nx,nu));
    sysd = c2d(sysc,sampling_period);
    A = [sysd.A, sysd.B; zeros(size(Bc,2),size(Bc,2)+size(Ac,2))];
    K = -dlqr(A,B,Q_ctrl,R_ctrl);

    F = zeros(nxt,nxt);
    G = zeros(nxt,nxt);
    H = zeros(nu,nxt);
    C_ext = eye(nxt);

end