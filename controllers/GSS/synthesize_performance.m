function res = synthesize_performance(sys,lmiSettings,bSilent)
%SYNTHESIZE PERFORMANCE
% For a given WH control system, this function synthesizes a state-feedback
% controller that renders the WH control system asymptotically stable
% despite deadline misses, and optimized l2-performance. The controller to
% be designed can be of switching or of non-switching nature, where
% switching controllers are able to stabilize a larger set of WH control
% systems and typically achieve better performance. The result will either
% be a stabilizing controller under which the closed-loop has guaranteed
% l2-performance, or in case the LMI optimization fails, no stabilizing
% state-feedback controller could be found. In this case, a stabilizing
% controller might not exist for the WH control system.

% INPUT
%   sys:    A struct defining the WH control system. See 'prepare_system.m'
%           for what is to be contained in this struct. For this analysis
%           relevant are the fields 'ss' and 'WHgraphs', containing the
%           (lifted) switched system representation and the associated WH
%           graphs, respectively.
%   lmiSettings: [optional] A struct containing settings for the LMI
%           solver. See 'check_LMI_settings.m' for what is can be contained
%           in this struct. If not specified, the default settings are
%           retrieved from the method 'check_LMI_settings.m'. For
%           synthesizing a non-switching controller, set
%           'lmiSettings.bVariableSlack' to false.
%
% OUTPUT
%   res:    A struct containing the results of the stability analysis with fields:
%           - sol:   The solution struct returned by the LMI solver
%           - l2:    The guaranteed l2-performance bound gamma
%           - G:     Optimal value of the slack variable G resulting from
%                    the LMI optimization
%           - S:     Optimal value of the Lyapunov matrices S
%                    (technically: inv(S_i)) resulting from the LMI
%                    optimization
%           - K_synth: The synthesized state-feedback controller gain(s) K.
%                    In case a switching controller has been designed, K is
%                    a struct with the fields v1, v2, ... corresponding to
%                    the controller gains of each node of the WH graph.
%           - graphType: The type of graph used for the analysis. Has
%                    merely informative purposes.
%           - edgeEigenvalues: The eigenvalues of the LMI associated with
%                    each edge (in order).


if ~bSilent
    disp('Synthesizing controller with guaranteed l2-performance...')
end

% Input consistency and defaults
if nargin < 2
    lmiSettings = struct;
end
lmiSettings = check_LMI_settings(lmiSettings);


%% Solve LMIs
[sol,l2,G,S,K,edgeEigenvalues] = solve_perf_LMI( ...
                    sys.WHgraphs.graph,true, ...
                    lmiSettings.bVariableSlack,sys.ss_synth,lmiSettings.tol, ...
                    lmiSettings.bSilent);

if sol.problem == 0
    if lmiSettings.bVariableSlack
        if ~bSilent
            disp(['  >> LMI problem successfully solved. A stabilizing switching controller has been found. Check solution struct for controller gain K. The achieved l2-performance is gamma = ', num2str(l2), '.'])
        end    
    else
        if ~bSilent
            disp(['  >> LMI problem successfully solved. A stabilizing non-switching controller has been found. Check solution struct for controller gain K. The achieved l2-performance is gamma = ', num2str(l2), '.'])
        end
    end
else
    warning('  >> LMI problem not solved. Check solver output for details. System might not be stabilizable by state feedback.')
end

% Combine all into a results struct
res.sol = sol;
res.l2 = l2;
res.G = G;
res.S = S;
res.K_synth = K;
res.graphType = 'lifted'; %save used graph type for vizualization later
res.edgeEigenvalues = edgeEigenvalues;

if ~bSilent
    disp(['Controller design completed.' newline])
end


end