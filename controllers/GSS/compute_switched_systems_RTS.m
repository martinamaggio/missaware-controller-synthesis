function [ss,ss_synth] = compute_switched_systems_RTS(sys,bSilent)
%COMPUTE SWITCHED SYSTEMS RTS
% This function computes up to three forms of the switched system
% formulation for the investigation of l2-performance for WH control
% systems. The closed-loop switched system is only available if a
% controller is provided in 'sys' to perform analysis. The (lifted)
% switched system representation for synthesis purposes is always computed.
% This functions automatically assumes a 1-step delay. Details see [1],
% e.g., for the construction of the matrices.
%
% INPUT
%   sys:    A struct containing all the WH control system properties. See
%           'prepare_system.m' for what is to be contained in this struct.
%           Note that sys.original.K can be empty if a controller is to be
%           synthesized. Analysis will then not be possible.
% 
% OUTPUT
%   ss:     The (lifted) switched system matrices for analysis purposes as a
%           struct with the (closed-loop) matrices A, Bw, C, Dw as fields,
%           being structs themself. They contain the fields 'mode0',
%           'mode1', ..., corresponding to the alpha-sequence. Example:
%           'ss.A.mode0', 'ss.Bw.mode3'.
%           Is only computed if a controller is provided.
%   ss_synth: The (lifted) system matrices for synthesis with the (open-loop)
%           matrices A, B, Bw, C, D, Dw as fields as in 'ss'.


%% Check input consistency
% Recover system matrices
A = sys.original.A;
B = sys.original.B;
Bw = sys.original.Bw;
C = sys.original.C;
D = sys.original.D;
Dw = sys.original.Dw;
K = sys.original.K;

% Check actuator and overrun strategy
if ~isfield(sys,'actuatorStrategy')
    error('The actuator strategy is not specified in sys.')
end
if ~isfield(sys,'overrunStrategy')
    error('The overrun strategy is not specified in sys.')
end

% Extract strategies
if sys.actuatorStrategy == "Zero" || sys.actuatorStrategy == "zero"
    actStrat = 'Zero';
elseif sys.actuatorStrategy == "Hold" || sys.actuatorStrategy == "hold"
    actStrat = 'Hold';
else
    error('Invalid actuator strategy specified in sys.')
end
if sys.overrunStrategy == "Kill" || sys.overrunStrategy == "kill"
    overStrat = 'Kill';
elseif sys.overrunStrategy == "Skip" || sys.overrunStrategy == "skip"
    overStrat = 'Skip';
else
    error('Invalid overrun strategy specified in sys.')
end

% Check if controller for analysis is provided, otherwise compute only
% switched system for synthesis
if any(any(isnan(K))) || any(isempty(K))
    if ~bSilent
        disp('     >> INFO: no controller is given, no analysis can be performed.')
    end
    bAnalysis = false;
else
    bAnalysis = true;
end

% Check matrices for dimension consistency
n = length(A);  % x
m = size(B,2);  % u
p = size(C,1);  % z
q = size(Bw,2); % w

if ~isequal(size(A),[n n])
    error('Matrix dimension(s) of A are not compatible.')
end
if ~isequal(size(B),[n m])
    error('Matrix dimension(s) of B are not compatible.')
end
if ~isequal(size(Bw),[n q])
    error('Matrix dimension(s) of Bw are not compatible.')
end
if ~isequal(size(C),[p n])
    error('Matrix dimension(s) of C are not compatible.')
end
if ~isequal(size(D),[p m])
    error('Matrix dimension(s) of D are not compatible.')
end
if ~isequal(size(Dw),[p q])
    error('Matrix dimension(s) of Dw are not compatible.')
end
if bAnalysis
    if isequal(size(K),[m n])
        % The provided controller might be designed for the non-augmented
        % state. Check dimension and add zeros if required.
        K = [K, zeros(m,m)];
        if ~bSilent
            disp(['     >> INFO: The provided controller does not use the',...
                      ' last applied input (usually because it was not',...
                      ' designed for a 1-step delay). Appending zeros to',...
                      ' match the augmented state.'])
        end
    elseif isequal(size(K),[m n+m])
        % correct size
    else
        error('Matrix dimension(s) of K are not compatible.')
    end
end

% Create mode list/check labels. Any number of modes, beginning with 0.
modeList = unique(sys.WHgraphs.graph.Edges.Weight); %already sorted
if any(modeList<0)
    error('The lifted graph contains negative edge labels (= mode names).')
end
if any(diff(modeList) ~= 1)
    error('The lifted graph contains non-consecutive edge labels (= mode names).')
end


%% Switched system matrices
% The matrices for both switched system representations are computed
% together, since the closed-loop matrices are easily computed from the
% open-loop switched system used for synthesis. Depending on the actuator
% and overrun strategy, the matrices differ.
% The detailed matrices can be found in [1], Figure 2.

for iMode = 1:length(modeList)
    alpha = modeList(iMode);
    modeName = ['mode',num2str(alpha)];

    % A_alpha
    sumA = zeros(n);
    if overStrat == "Skip" && actStrat == "Hold"
        for ii=0:alpha
            sumA = sumA + A^ii;
        end
    else
        sumA = A^alpha;
    end
    A_synth = [A^(alpha+1), sumA*B;...
               zeros(m,n), zeros(m,m)];
    clear sumA

    % B_alpha
    if alpha==0 || overStrat == "Skip"
        B_synth = [zeros(n,m); eye(m)];
    else
        sumA = zeros(n);
        if actStrat == "Zero"
            B_synth = [A^(alpha-1)*B; zeros(m,m)];
        else %Hold
            for ii=0:alpha-1
                sumA = sumA + A^ii;
            end
            B_synth = [sumA*B; eye(m)];
            clear sumA
        end
    end

    % C_alpha
    C_synth = [C, D];
    if alpha >= 1
        sumA = zeros(n);
        for ii=0:alpha-1
            if overStrat == "Skip" && actStrat == "Hold"
                sumA = sumA + A^ii;
                C_synth = [C_synth;...
                           C*A^(ii+1), D + C*sumA*B];
            else
                sumA = A^ii;
                C_synth = [C_synth;...
                           C*A^(ii+1), C*sumA*B];
            end
        end
    end

    % D_alpha
    if overStrat == "Skip"
        D_synth = zeros((alpha+1)*p,m);
    else
        switch alpha
            case 0
                D_synth = zeros(p,m);
            case 1
                D_synth = [zeros(p,m); D];
            otherwise
                D_synth = [zeros(p,m); D];
                sumA = zeros(n);
                for ii=0:alpha-2
                    if actStrat == "Zero"
                        sumA = A^ii;
                        D_synth = [D_synth; C*sumA*B];
                    else % hold strategy
                        sumA = sumA + A^ii;
                        D_synth = [D_synth; D + C*sumA*B];
                    end
                    
                end
        end
    end

    % Bw and Dw - same for all strategy combinations
    Bw_lif = A^alpha * Bw;
    for ii=alpha-1:-1:0
        Bw_lif = [Bw_lif, A^ii * Bw];
    end
    Bw_lif = [Bw_lif; zeros(m,(alpha+1)*q)];

    % Lifted Dw - row by row
    if alpha == 0
        Dw_lif = Dw;
    else
        Dw_lif = [Dw, zeros(p,alpha*q)];
        for ii=0:alpha-1
            D_loop = C * A^ii * Bw;
            for jj=ii-1:-1:0
                D_loop = [D_loop, C * A^jj * Bw];
            end
            D_loop = [D_loop, Dw, zeros(p,(alpha-ii-1)*q)];
            Dw_lif = [Dw_lif; D_loop];
        end
    end

    % Assign matrices to switched system structs
    ss_synth.A.(modeName) = A_synth;
    ss_synth.B.(modeName) = B_synth;
    ss_synth.Bw.(modeName) = Bw_lif;
    ss_synth.C.(modeName) = C_synth;
    ss_synth.D.(modeName) = D_synth;
    ss_synth.Dw.(modeName) = Dw_lif;
    if bAnalysis
        % Closed-loop sytem matrices for lifted analysis from open-loop
        % (synthesis) matrices
        ss.A.(modeName) = A_synth + B_synth * K;
        ss.Bw.(modeName) = Bw_lif;
        ss.C.(modeName) = C_synth + D_synth * K;
        ss.Dw.(modeName) = Dw_lif;
    end
end

if bAnalysis
    ss.B = []; % no input for closed-loop system
    ss.D = [];
else % if no controller is provided, return only synthesis system
    ss = NaN;
end


end