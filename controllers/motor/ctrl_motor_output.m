% CTRL_MOTOR_OUTPUT designs a linear dynamic output feedback controller for an electric motor
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [F,G,H,K,C_ext] = ctrl_motor_output(sampling_period,C)
% Designs a linear dynamic output feedback controller for the linear model 
% of the electric motor with one-step-delayed actuation.
%
% Inputs
%   sampling_period : sample time
%   C               : output matrix
% 
% Outputs
%   F,G,H,K         : matrices of the dynamic controller:
%                       z[k+1] = F z[k] + G y[k]
%                       u[k+1] = H z[k] + K y[k]
%   C_ext           : extended output matrix for one-step-delayed actuation

    [Ac,Bc] = motor_matrices();
    nx = size(Ac,1);
    nu = size(Bc,2);
    ny = size(C,1);

    % constant discrete-time matrices of extended system
    % where x_extended(k) = [x(k); u^c(k-1)] models a one-step delay
    D = zeros(ny,nu);
    C_ext = [C, D];
    B = [zeros(nx,nu); eye(nu)];

    Q_ctrl = diag([30,30,10,0,0]);
    R_ctrl = diag([20,20]);
    Q_obs = 5*Q_ctrl;
    R_obs = R_ctrl;

    sysc = ss(Ac,Bc,C,D);
    sysd = c2d(sysc,sampling_period);
    A = [sysd.A, sysd.B; zeros(size(Bc,2),size(Bc,2)+size(Ac,2))];
    K = dlqr(A,B,Q_ctrl,R_ctrl);
    L = dlqr(A',C_ext',Q_obs,R_obs)';

    F = A - L*C_ext - B*K;
    G = L;
    H = -K;
    K = zeros(1, size(C,1));
end