% RUN_COMMON_SEQUENCE_FURUTA runs the furuta pendulum with all controllers and strategies
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

% Figures 1 - 3 of the paper are created using the data generated when running this script. 
% This runs all controller/strategy combinations with common simulation settings 
% for the Furuta pendulum.

clear; clc;
close all;
rng(10);
tic

addpath(genpath(pwd));

% parameters of the system and simulation *******************************************
psys.plant_name = 'furuta';                     % plant
psys.plant_sim = 'nonlinear';                   % {'nonlinear', 'linear'} simulation model
psys.period = 0.005;                            % controller period (seconds)
psys.x0 = [pi; 0; 0];                           % initial state (angle, angular velocity, base velocity)
psys.u0 = 0;                                    % initial control signal
psys.n_mc = 1;                                  % number of Monte Carlo runs
psys.C = eye(length(psys.x0));                  % output matrix (full state feedback)

% parameters of the real-time setting and deadline miss model
% generation of sequence of deadline misses
duration = 2.5;                                 % simulation duration (seconds)
num_iterations = ceil(duration/psys.period);    % number of time steps of simulation
p_miss = 0.9;                                   % probability of a deadline miss of free iterations
prt.wh_constraint = 'any_miss';                 % type of weakly hard constraint
prt.m = 3; prt.w = 5;                           % parameters of wh constraint
prt.h = prt.w-prt.m; 
prt.hm_sequence = deadline_anyhit_mandatory(num_iterations,0,prt.h,prt.w,p_miss);
prt.sample_sequence = deadline_anyhit_mandatory(1e3,0,prt.h,prt.w,p_miss);

% white process noise
process_noise_std_dev = [0.01; 0.0; 0.0];
process_noise_seq = process_noise_std_dev .* randn(length(psys.x0),num_iterations,psys.n_mc);
% brown process noise
s = dsp.ColoredNoise('brown',num_iterations,1)';
process_noise_seq(1,:) = 0.0004*s();
% kick disturbance sequence
kick_seq = zeros(size(process_noise_seq));
kick_seq(3,1.5/psys.period:1.5/psys.period+1) = 4;
psys.quantile_val = []; % quantiles to be computed for states, inputs, metrics

% list of metrics that are computed for all controllers
metrics = {
    struct('name', 'pendulum position upward', 'function', @(x,u,t) metric_furuta_stabilized(x));
    struct('name', 'mean absolute error of x', 'function', @(x,u,t) metric_mae(x,u,'x',psys.plant_name,psys.quantile_val));
    struct('name', 'mean absolute input u', 'function', @(x,u,t) metric_mae(x,u,'u',psys.plant_name,psys.quantile_val));
    struct('name', 'offline computation time', 'function', @(x,u,t) struct('value',t(1)));
    struct('name', 'online computation time', 'function', @(x,u,t) metric_computation_time(t(2:end),psys.quantile_val));
    struct('name', 'mean square error of x', 'function', @(x,u,t) metric_mse(x,u,'x',psys.plant_name,psys.quantile_val));
    };


%% Kill & zero experiment
prt.overrun_strategy = "kill";
prt.actuator_strategy = "zero";

controllers_kill_zero = {
        struct('method', 'graph_lqr', 'function', @() setup_graph_lqr(psys,prt));
        struct('method', 'dmac', 'function', @() setup_dmac(psys,prt));
        struct('method', 'dmai', 'function', @() setup_dmai(psys,prt,@ctrl_furuta_linear));
        struct('method', 'dr_mjls', 'function', @() setup_drmjls(psys,prt,true));
        struct('method', 'tree_mpc', 'function', @() setup_treempc(psys,prt));
        struct('method', 'gss', 'function', @() setup_gss(psys,prt,true));
        struct('method', 'baseline', 'function', @() deal(@furuta_original, []));
    };

do_experiment('kill_zero/disturbance',controllers_kill_zero,metrics,psys,prt,kick_seq);
do_experiment('kill_zero/noise',controllers_kill_zero,metrics,psys,prt,process_noise_seq);

%% Kill & hold experiment
prt.overrun_strategy = "kill";
prt.actuator_strategy = "hold";

controllers_kill_hold = {
        struct('method', 'acc_control', 'function', @() setup_acc_control(psys,prt));
        struct('method', 'dmac', 'function', @() setup_dmac(psys,prt));
        struct('method', 'dmai', 'function', @() setup_dmai(psys,prt,@ctrl_furuta_linear));
        struct('method', 'dr_mjls', 'function', @() setup_drmjls(psys,prt,true));
        struct('method', 'tree_mpc', 'function', @() setup_treempc(psys,prt));
        struct('method', 'gss', 'function', @() setup_gss(psys,prt,true));
        struct('method', 'baseline', 'function', @() deal(@furuta_original, []));
    };

do_experiment('kill_hold/disturbance',controllers_kill_hold,metrics,psys,prt,kick_seq);
do_experiment('kill_hold/noise',controllers_kill_hold,metrics,psys,prt,process_noise_seq);

%% Skip & zero experiment
prt.overrun_strategy = "skip";
prt.actuator_strategy = "zero";

controllers_skip_zero = {
        struct('method', 'dmac', 'function', @() setup_dmac(psys,prt));
        struct('method', 'dr_mjls', 'function', @() setup_drmjls(psys,prt,true));
        struct('method', 'tree_mpc', 'function', @() setup_treempc(psys,prt));
        struct('method', 'gss', 'function', @() setup_gss(psys,prt,true));
        struct('method', 'baseline', 'function', @() deal(@furuta_original, []));
    };

do_experiment('skip_zero/disturbance',controllers_skip_zero,metrics,psys,prt,kick_seq);
do_experiment('skip_zero/noise',controllers_skip_zero,metrics,psys,prt,process_noise_seq);

%% Skip & hold experiment
prt.overrun_strategy = "skip";
prt.actuator_strategy = "hold";

controllers_skip_hold = {
        struct('method', 'dmac', 'function', @() setup_dmac(psys,prt));
        struct('method', 'streamadapt', 'function', @() setup_streamadapt(psys,prt,'state'));
        struct('method', 'dr_mjls', 'function', @() setup_drmjls(psys,prt,true));
        struct('method', 'tree_mpc', 'function', @() setup_treempc(psys,prt));
        struct('method', 'gss', 'function', @() setup_gss(psys,prt,true));
        struct('method', 'baseline', 'function', @() deal(@furuta_original, []));
    };

do_experiment('skip_hold/disturbance',controllers_skip_hold,metrics,psys,prt,kick_seq);
do_experiment('skip_hold/noise',controllers_skip_hold,metrics,psys,prt,process_noise_seq);

toc