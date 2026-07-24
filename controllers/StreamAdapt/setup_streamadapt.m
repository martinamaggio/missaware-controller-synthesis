% SETUP_STREAMADAPT implements the StreamAdapt state or output feedback controller
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [ctrl_func, z0] = setup_streamadapt(psys,prt,ctrl_type)
% Implements the StreamAdapt state or output feedback controller
% for a worst case reaction time of WCRT=(m+1)*period.
% The sensor period is assumed to correspond with the control period.
%
% Inputs
%   psys                : struct containing the system and simulation parameters
%   prt                 : struct of the real-time setting parameters
%   ctrl_type           : controller type ('state' or 'output' feedback)
%
% Outputs
%   ctrl_func           : control function wrapper
%   z0                  : initial controller state (stateless: z0=[])


    % Extract the needed system and real-time parameters
    plant_name = psys.plant_name;               % name of plant ('furuta' or 'motor' implemented)
    period = psys.period;                       % control period
    C = psys.C;                                 % output matrix
    m = prt.m;                                  % rowMiss(m) parameter
    strategy_overrun = prt.overrun_strategy;    % overrun strategy (must be 'skip')
    strategy_actuator = prt.actuator_strategy;  % actuator strategy (must be 'hold')
    
    uses_skip_hold = strcmp(strategy_overrun,'skip') && strcmp(strategy_actuator,'hold');
    if ~uses_skip_hold
        error('StreamAdapt controller assumes a skip and hold overrun/actuator strategy')
    end
    
    % retrieve plant specific parameters and nominal control design function
    [Ac,Bc,C,design_func] = feval(['streamadapt_param_',plant_name],C);

    nx = size(Ac,1);
    nu = size(Bc,2);
    ny = size(C,1);

    % constant discrete-time matrices of extended system
    % where x_extended(k) = [x(k); u(k-1)] models a one-step delay
    D = zeros(ny,nu);
    C_ext = [C, D];
    B = [zeros(nx,nu); eye(nu)];

    % Discrete-time extended system matrix A, state or output feedback gain K
    % and observer gain L are computed for every possible control job time,
    % bounded by the WCRT.
    A = cell(1,m+1);
    K = cell(1,m+1);
    L = cell(1,m+1);
    for i = 0:m
        Ti = (i+1)*period;
        A{i+1} = compute_A_ext_discrete(Ac,Bc,C,D,Ti);
        [K{i+1},L{i+1}] = design_func(A{i+1},B,C_ext,Ti);
    end

    % function wrapper for state feedback control
    function [u,z] = ctrl_wrapper_statefb(x,z,u,sequence,~)
        % Computes the control signal using state feedback switched
        % depending on the previous job's computation time.
        %
        % Inputs
        %   x           : system state
        %   z           : old controller state (not needed)
        %   u           : old system input
        %   sequence    : sequence of job outcomes up to this job's termination
        % Outputs
        %   u           : new system input
        %   z           : new controller state (not needed)

        l = last_job_duration(sequence);
        u = - K{l} * [x;u];
    end

    % function wrapper for output feedback control
    function [u,z] = ctrl_wrapper_outputfb(x,z,~,sequence,~)
        % Estimates the state and computes the control signal using an
        % observer and feedback controller switched depending on the
        % previous job's computation time.
        %
        % Inputs
        %   x           : system state (to be converted into measured output)
        %   z           : old estimated extended state
        %   sequence    : sequence of job outcomes up to this job's termination
        % Outputs
        %   u           : new system input
        %   z           : new estimated extended state

        l = last_job_duration(sequence);
        y = C*x;
        u = - K{l} * z;
        z = (A{l} - L{l}*C_ext - B*K{l})*z + L{l}*y;
    end

    switch ctrl_type
        case 'state'
            ctrl_func = @ctrl_wrapper_statefb;
            z0 = [];
        case 'output'
            ctrl_func = @ctrl_wrapper_outputfb;
            z0 = zeros(size(K{1},2),1);
        otherwise
            error('Controller type not implemented')
    end
end

function dur = last_job_duration(sequence)
    % Extracts the number of sample periods of the previous job computation.
    % The last 1 (indicating a finished job) is mapped to the ongoing job, 
    % thus the previous job's duration is extracted using the two hits before. 
    idx_hit = find(sequence==1,3,'last');
    dur = idx_hit(2)-idx_hit(1);
end

function A = compute_A_ext_discrete(Ac,Bc,C,D,T)
    % Computes the extended discrete-time system matrix for a one-step delayed input.
    sysc = ss(Ac,Bc,C,D);
    sysd = c2d(sysc,T);
    A = [sysd.A, sysd.B; zeros(size(Bc,2),size(Bc,2)+size(Ac,2))];
end
