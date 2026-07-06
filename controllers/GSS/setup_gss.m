function [ctrl_fcn, z0] = setup_gss(psys,prt,bSwitching)


%% Init
% Sets up the computation environment, like adding required folders to the
% MATLAB path and making sure that the optimization etc... works.
bSilent = true;

if ~bSilent
    disp('Initializing GSS controller...')
end
format short

% add paths
%disp('    adding paths...')
%addpath('C:/Program Files/mosek/10.0/toolbox/r2017a')

if ~bSilent
    disp('Initialization done.')
end

% extract parameters
plant_name = psys.plant_name;
period = psys.period;
WHconstraintType = prt.wh_constraint;
m = prt.m; k = prt.w; h = prt.h;
strategy_overrun = prt.overrun_strategy;
strategy_actuator = prt.actuator_strategy;

%% Define system - Furuta pendulum
system = feval(['gss_param_',plant_name],period,strategy_actuator,strategy_overrun);

% Build WH constraint
if strcmp(WHconstraintType,'any_miss')
    WH_constraint = any_miss(m,k);
elseif strcmp(WHconstraintType,'any_hit')
    WH_constraint = any_hit(h,k);
elseif strcmp(WHconstraintType,'row_miss')
    WH_constraint = row_miss(m);
elseif strcmp(WHconstraintType,'row_hit')
    WH_constraint = row_hit(h,k);
else
    error('Invalid WH constraint type specified.')
end

% overrun strategy: Kill, Skip
% actuator strategy: Zero or Hold
overrunStrategy = strategy_overrun;
actuatorStrategy = strategy_actuator;


%% Prepare system
% Runs all the necessary preparation computations for the analysis and
% synthesis later. Specifically, it computes the required WH graphs, checks
% and includes the actuator and loss strategies, and computes the required
% switched system representations.

if ~bSilent
    disp('Preparing switched systems...')
end

% Compute WH graphs
sys.WHgraphs = compute_WH_graphs(WH_constraint,false,true,overrunStrategy,bSilent);

% Check consistency of settings
if ~bSilent
    fprintf('    Checking settings...')
end

% actuator strategy
actuatorStrategy = convertStringsToChars(actuatorStrategy);
if ~ismember(actuatorStrategy,{'hold','Hold','zero','Zero'})
    error('Invalid actuator strategy specified.')
end

% overrun strategy
overrunStrategy = convertStringsToChars(overrunStrategy);
if strcmp(overrunStrategy,'queue') || strcmp(overrunStrategy,'Queue')
    error('GSS does not support the Queue strategy.')
end
if ~ismember(overrunStrategy,{'Kill','Skip','kill','skip'})
    error('Invalid overrun strategy specified.')
end

if ~bSilent
    disp(' valid!')
end

% Display system information
if ~bSilent
    disp(['    System properties:' newline '      * 1-step delay' newline ...
          '      * actuator strategy: ', actuatorStrategy, newline ...
          '      * overrun strategy: ', overrunStrategy, newline ...
          '      * WH constraint: ', WH_constraint.type, '(', ...
           num2str(WH_constraint.r), ',', num2str(WH_constraint.s) ,')'])
end


%% Compute switched system representation
% Save specified system as original system in the sys-struct
sys.original = system;
sys.overrunStrategy = overrunStrategy;
sys.actuatorStrategy = actuatorStrategy;
clear system overrunStrategy actuatorStrategy

if ~bSilent
    disp('    Computing switched systems...')
end

[sys.ss,sys.ss_synth] = compute_switched_systems_RTS(sys,bSilent);

if ~bSilent
    disp(['Switched system preparations done.' newline])
end



%% Synthesis of controller
K_s = NaN; K_ns = NaN;
if bSwitching
    if ~bSilent
        disp('### Synthesis of switching controller ###')
    end
    settings.bSwitched = true;
    res_s = synthesize_performance(sys,settings,bSilent);
    if ~bSilent
        disp('Switching controller synthesis result:')
        K_s = res_s.K_synth
    end
else % non-switching controller
    if ~bSilent
        disp('### Synthesis of non-switching controller ###')
    end
    settings.bSwitched = false;
    res_ns = synthesize_performance(sys,settings,bSilent);
    if ~bSilent
        disp('Non-switching controller synthesis result:')
        disp(['K_ns = [',num2str(round(res_ns.K_synth,4)),']',newline])
    end
end

function [u,z] = ctrl_wrapper_ns(x,z,u,~)   
    u = res_ns.K_synth * [x; u];
end

function [u,z] = ctrl_wrapper_s(x,z,u,sequence,~)
    %disp(['DEBUG:    original sequence: ',num2str(sequence)])

    % Pick correct node for controller
    sequence = char(strjoin(string(sequence),''));

    % Always remove last enty (which is alsways a 1) to match node description of theory
    sequence = sequence(1:end-1);

    % For Skip, additionally remove all entries after last hit, since those are unknown
    if strcmp(sys.overrunStrategy,'skip') || strcmp(sys.overrunStrategy,'Skip')
        idx_hit = find(sequence=='1');
        if ~isempty(idx_hit)
            sequence = sequence(1:idx_hit(end));
        else
            sequence = '';
        end
    end

    for ii = length(sys.WHgraphs.nodeLabels_lif):-1:1 %start from the back to ensure that the longest matching sequence is found
        node2check = sys.WHgraphs.nodeLabels_lif{ii};
        node2check = node2check(~isspace(node2check)); %remove whitespace
        if endsWith(sequence,node2check)
            K = res_s.K_synth.(['v',num2str(ii)]);
            break
        end
    end
    %disp(['DEBUG:    Controller node: ',num2str(ii),', sequence: ',node2check])
    try
        u = K * [x; u];
    catch
        error('Controller for this node not defined, something went wrong!')
    end
end

if bSwitching
    ctrl_fcn = @ctrl_wrapper_s;
else
    ctrl_fcn = @ctrl_wrapper_ns;
end

z0 = [];

end