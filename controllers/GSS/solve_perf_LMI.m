function [sol,l2_gain,G,S,K,minEigenvaluesEdges] = solve_perf_LMI(graph,bSynth,bVarG,ss,tol,bSilent)
%SOLVE PERFORMANCE LMI (Linear Matrix Inequality)
% This function solves the l2-performance LMI for a switched system, which
% switching is restricted by an automaton represented by a graph. For
% solving, YALMIP and the solver MOSEK are used. Both have to be available
% and installed.
% The function offers analysis and synthesis possibilities:
%   - Analysis: For a given (potentially closed-loop) switched system,
%     determine an upper bound on the l2-gain for the system.
%   - Synthesis: Given a switched system, synthesize a controller such that
%     the resulting l2-gain is minimized.
% Further, both analysis and synthesis can be computed with varying
% sdp variables G_i for a switched controller (less conservative), as well
% as fixed G_i = G for a non-switched controller.
%
% INPUT
%   graph:    A MATLAB digraph object representing the constrained
%             switching with the fields:
%             graph.Edges: A table containing the edge definitions in the
%                          column EndNodes defined as [tail, head], where
%                          tail is where the arrow starts and the
%                          corresponding weights in the Weight column,
%                          specifying the switched system mode for each
%                          edge.
%                  .Nodes: Can be empty, not required
%             Example: graph.Edges = EndNodes    Weight
%                                    ________    ______
%                                     1    1       1   
%                                     1    2       0   
%                                     2    1       1   
%                      representing the WHRTC (1,2)
%   bSynth:   Boolean variable specifying if analysis or controller
%             synthesis is to be performed. See also the implications on
%             B and D for analysis.
%   bVarG:    Boolean variable specifying if variable G_i should be used
%             in the LMI (analysis and synthesis).
%   ss:       The switched system as a struct containing the
%             corresponding matrices. If analysis is performed, no B and
%             D is needed, therefore set them to '[]'.
%             Dimensions:
%                 A:  (n x n)
%                 B:  (n x m)
%                 Bw: (n x q)
%                 C:  (p x n)
%                 D:  (p x m)
%                 Dw: (p x q)
%   tol:      [optional] The LMI contraint tolerance. Default: 1e-7.
%   bSilent:  [optional] If true, suppresses output of solver status.
%
% OUTPUT
%   sol:      The MOSEK solution struct with the solution details.
%   l2_gain:  The achieved l2-gain as a double.
%   G,S:      Matrix variables of the LMI. inv(S) can be used as a switched
%             Lyapunov function. 
%   K:        The resulting synthesized controller. Returns 'NaN' if
%             analysis is performed. If variable G_i are used, it returns a
%             struct for the switched controller K_i.
%   minEigenvaluesEdges: The minimum eigenvalue of the corresponding LMI
%             constraint of all edges in order of the edge definition. Can
%             be used to check constraint satisfaction.


% Set defaults if necessary
if nargin < 5
    tol = 1e-7;
end
if nargin < 6
    bSilent = false;
end

% Initializing
numEdges = size(graph.Edges.EndNodes,1);
minEigenvaluesEdges = nan(numEdges,1);


%% Check input consistency
% Check if switched system exists
if ( isstruct(ss) && isempty(fieldnames(ss)) ) || ( ~isstruct(ss) && isnan(ss) )
    error('The provided switched system is empty or NaN. Did you try to do analysis without specifying a controller?')
end

% Recover system matrices
A = ss.A;
B = ss.B;
Bw = ss.Bw;
C = ss.C;
D = ss.D;
Dw = ss.Dw;

A_names = fieldnames(A);
Bw_names = fieldnames(Bw);
C_names = fieldnames(C);
Dw_names = fieldnames(Dw);

% Check number of modes defined
if ~isequal(size(A_names),size(Bw_names)) || ...
       ~isequal(size(Bw_names),size(C_names)) || ...
       ~isequal(size(C_names),size(Dw_names))
    error('There are a different number of modes for the system matrices defined.')
end

% Check if mode names are consistent
modeNames = intersect(intersect(intersect(A_names,Bw_names),C_names),Dw_names);
if length(modeNames) ~= length(A_names)
    error('Mode names of matrices are not consistent.')
end

% Check matrices for dimension consistency
for iMode = 1:length(modeNames)
    mode = modeNames{iMode};

    n = length(A.(mode));
    p = size(C.(mode),1);
    q = size(Bw.(mode),2);

    if ~isequal(size(A.(mode)),[n n])
        error(['Matrix dimension of A in field ',mode,' is not compatible.'])
    end
    if ~isequal(size(Bw.(mode)),[n q])
        error(['Matrix dimension of Bw in field ',mode,' is not compatible.'])
    end
    if ~isequal(size(C.(mode)),[p n])
        error(['Matrix dimension of C in field ',mode,' is not compatible.'])
    end
    if ~isequal(size(Dw.(mode)),[p q])
        error(['Matrix dimension of Dw in field ',mode,' is not compatible.'])
    end
end

% For synthesis matrices B and D, also check consistency. Same as above.
if bSynth
    if isempty(B) || isempty(D)
        error('B and D matrices cannot be empty for controller synthesis.')
    else
        B_names = fieldnames(B);
        D_names = fieldnames(D);
    end
    if ~isequal(size(A_names),size(B_names)) || ~isequal(size(B_names),size(D_names))
        error('There are a different number of modes for the system matrices defined.')
    end
    modeNames = intersect(intersect(B_names,D_names),modeNames);
    if length(modeNames) ~= length(A_names)
        error('Mode names of matrices are not consistent.')
    end
    for iMode = 1:length(modeNames)
        mode = modeNames{iMode};
        n = length(A.(mode));
        p = size(C.(mode),1);
        m = size(B.(mode),2);
        if ~isequal(size(B.(mode)),[n m])
            error(['Matrix dimension of B in field ',mode,' is not compatible.'])
        end
        if ~isequal(size(D.(mode)),[p m])
            error(['Matrix dimension of D in field ',mode,' is not compatible.'])
        end
    end
end


%% Set up optimization problem
yalmip('clear');

% Create sdpvars for LMIs
ga = sdpvar(1);
for iVertex = 1:numnodes(graph)
    S.(['v',num2str(iVertex)]) = sdpvar(n,n,'symmetric'); %each node requires a separate S_i
    if bVarG %variable G_i and R_i
        G.(['v',num2str(iVertex)]) = sdpvar(n,n,'full');
        if bSynth
            R.(['v',num2str(iVertex)]) = sdpvar(m,n,'full');
        end
    else %constant G and R
        G = sdpvar(n,n,'full');
        if bSynth
            R = sdpvar(m,n,'full');
        end
    end
end

% Build the LMIs
constrLMIs = lmi('');
for iEdge = 1:numEdges

    % Obtain the correct edge information from the graph
    i = graph.Edges.EndNodes(iEdge,1);
    j = graph.Edges.EndNodes(iEdge,2);
    l = graph.Edges.Weight(iEdge);
    mode = ['mode',num2str(l)];

    % Update matrix dimensions (may have changed due to mode)
    n = length(A.(mode));
    p = size(C.(mode),1);
    q = size(Bw.(mode),2);

    % Get corresponding matrices for the mode l
    Al = A.(mode);
    Blw = Bw.(mode);
    Cl = C.(mode);
    Dlw = Dw.(mode);
    
    % Add the corresponding LMI
    if ~bVarG %constant matrices G and R
        if bSynth % Synthesis
            Bl = B.(mode);
            Dl = D.(mode);
            M = [G+G'-S.(['v',num2str(i)]), zeros(q,n)', G'*Al'+R'*Bl'       , G'*Cl'+R'*Dl';...
                 zeros(q,n)               , ga*eye(q)  , Blw'                , Dlw' ;...
                 Al*G+Bl*R                , Blw        , S.(['v',num2str(j)]), zeros(p,n)';...
                 Cl*G+Dl*R                , Dlw        , zeros(p,n)          , ga*eye(p)];
        else % Analysis
            M = [G+G'-S.(['v',num2str(i)]), zeros(q,n)', G'*Al'              , G'*Cl';...
                 zeros(q,n)               , ga*eye(q)  , Blw'                , Dlw' ;...
                 Al*G                     , Blw        , S.(['v',num2str(j)]), zeros(p,n)';...
                 Cl*G                     , Dlw        , zeros(p,n)          , ga*eye(p)];
        end
    else %variable matrices G_i and R_i
        if bSynth % Synthesis
            vertex = ['v',num2str(i)];
            Bl = B.(mode);
            Dl = D.(mode);
            M = [G.(vertex)+G.(vertex)'-S.(vertex), zeros(q,n)', G.(vertex)'*Al'+R.(vertex)'*Bl', G.(vertex)'*Cl'+R.(vertex)'*Dl';...
                 zeros(q,n)                       , ga*eye(q)  , Blw'                           , Dlw' ;...
                 Al*G.(vertex)+Bl*R.(vertex)      , Blw        , S.(['v',num2str(j)])           , zeros(p,n)';...
                 Cl*G.(vertex)+Dl*R.(vertex)      , Dlw        , zeros(p,n)                     , ga*eye(p)];
        else % Analysis
            vertex = ['v',num2str(i)]; 
            M = [G.(vertex)+G.(vertex)'-S.(vertex), zeros(q,n)', G.(vertex)'*Al'     , G.(vertex)'*Cl';...
                 zeros(q,n)                       , ga*eye(q)  , Blw'                , Dlw' ;...
                 Al*G.(vertex)                    , Blw        , S.(['v',num2str(j)]), zeros(p,n)';...
                 Cl*G.(vertex)                    , Dlw        , zeros(p,n)          , ga*eye(p)];
        end
    end
    constrLMIs = [constrLMIs, M >= tol];
end


%% Solve the LMI
if bSilent
    options = sdpsettings('solver','mosek','verbose',0,'debug',1);
    sol = optimize(constrLMIs,ga,options);
else
    options = sdpsettings('solver','mosek','verbose',1,'debug',1);
    sol = optimize(constrLMIs,ga,options)
end

% obtain numerical values from sdpvars
l2_gain = value(ga);
for iVertex = 1:numnodes(graph)
    S.(['v',num2str(iVertex)]) = value(S.(['v',num2str(iVertex)]));
end
if bVarG
    for iVertex = 1:numnodes(graph)
        G.(['v',num2str(iVertex)]) = value(G.(['v',num2str(iVertex)]));
    end
else
    G = value(G);
end

% synthesize controller
if bSynth
    if bVarG
        for iVertex = 1:numnodes(graph)
            R.(['v',num2str(iVertex)]) = value(R.(['v',num2str(iVertex)]));
            K.(['v',num2str(iVertex)]) = R.(['v',num2str(iVertex)]) * inv(G.(['v',num2str(iVertex)]));
        end
    else
        R = value(R);
        K = value(R)*inv(G);
    end
else
    K = NaN;
end


%% Final feasibility test
if sol.problem == 1 || sol.problem == 4 % infeasible problem or numerical problems
    return
end

return % skip feasibility test, only required for debugging

for iEdge = 1:numEdges

    % Obtain the correct edge information from the graph
    i = graph.Edges.EndNodes(iEdge,1);
    j = graph.Edges.EndNodes(iEdge,2);
    l = graph.Edges.Weight(iEdge);
    mode = ['mode',num2str(l)];

    % Update matrix dimensions (may have changed due to mode)
    n = length(A.(mode));
    p = size(C.(mode),1);
    q = size(Bw.(mode),2);

    % Get corresponding matrices for the mode l
    Al = A.(mode);
    Blw = Bw.(mode);
    Cl = C.(mode);
    Dlw = Dw.(mode);
    
    % Look for the corresponding LMI
    if ~bVarG %constant matrices G and R
        if bSynth % Synthesis
            Bl = B.(mode);
            Dl = D.(mode);
            M = [G+G'-S.(['v',num2str(i)]), zeros(q,n)'   , G'*Al'+R'*Bl'       , G'*Cl'+R'*Dl';...
                 zeros(q,n)               , l2_gain*eye(q), Blw'                , Dlw' ;...
                 Al*G+Bl*R                , Blw           , S.(['v',num2str(j)]), zeros(p,n)';...
                 Cl*G+Dl*R                , Dlw           , zeros(p,n)          , l2_gain*eye(p)];
        else % Analysis
            M = [G+G'-S.(['v',num2str(i)]), zeros(q,n)'   , G'*Al'              , G'*Cl';...
                 zeros(q,n)               , l2_gain*eye(q), Blw'                , Dlw' ;...
                 Al*G                     , Blw           , S.(['v',num2str(j)]), zeros(p,n)';...
                 Cl*G                     , Dlw           , zeros(p,n)          , l2_gain*eye(p)];
        end
    else %variable matrices G_i and R_i
        if bSynth % Synthesis
            vertex = ['v',num2str(i)];
            Bl = B.(mode);
            Dl = D.(mode);
            M = [G.(vertex)+G.(vertex)'-S.(vertex), zeros(q,n)'   , G.(vertex)'*Al'+R.(vertex)'*Bl', G.(vertex)'*Cl'+R.(vertex)'*Dl';...
                 zeros(q,n)                       , l2_gain*eye(q), Blw'                           , Dlw' ;...
                 Al*G.(vertex)+Bl*R.(vertex)      , Blw           , S.(['v',num2str(j)])           , zeros(p,n)';...
                 Cl*G.(vertex)+Dl*R.(vertex)      , Dlw           , zeros(p,n)                     , l2_gain*eye(p)];
        else % Analysis
            vertex = ['v',num2str(i)];
            M = [G.(vertex)+G.(vertex)'-S.(vertex), zeros(q,n)'   , G.(vertex)'*Al'     , G.(vertex)'*Cl';...
                 zeros(q,n)                       , l2_gain*eye(q), Blw'                , Dlw' ;...
                 Al*G.(vertex)                    , Blw           , S.(['v',num2str(j)]), zeros(p,n)';...
                 Cl*G.(vertex)                    , Dlw           , zeros(p,n)          , l2_gain*eye(p)];
        end
    end
    eigLMI = eig(M);

    % Find active constraints: look for smallest eigenvalue for each edge
    minEigenvaluesEdges(iEdge) = min(eigLMI);

    % Check constraint satisfaction
    if any(eigLMI <= 0)
        disp(['  >> WARNING: Final feasibility test failed in edge ',num2str(iEdge),'. The eigenvalues of the LMI are:'])
        eigLMI
    end
end

end