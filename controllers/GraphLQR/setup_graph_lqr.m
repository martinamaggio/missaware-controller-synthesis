function [ctrl_fcn, z0] = setup_graph_lqr(psys,prt)

% Notes
% - restriction: only Kill and Zero
% - implemented is M3 of the paper (not the more conservative iterative method M4)

plant_name = psys.plant_name;
period = psys.period;
h = prt.h;
k = prt.w;
strategy_overrun = prt.overrun_strategy;
strategy_actuator = prt.actuator_strategy;

if ~(strategy_overrun=="kill")
    error('Graph-LQR controller assumes a kill overrun strategy.')
end
if ~(strategy_actuator=="zero")
    error('Graph-LQR controller assumes the zero actuator strategy.')
end


%% Define system
% plant specific parameters
[A_aug,B_aug,Q,S,R] = feval(['graphlqr_param_',plant_name],period);


%% Prepare system
% Compute WH graph
graph = create_any_WH_graph(h,k); %using code from GSS
[graph,nodeLabels] = adapt_graph(graph);

% figure
% plot(graph)

K_synth = synthesize_graph_lqr_ctrl(A_aug,B_aug,Q,S,R,graph);

%--
function [u,z] = ctrl_wrapper(x,z,u,sequence,~)
    %disp(['DEBUG:    original sequence: ',num2str(sequence)])

    % Pick correct node for controller
    sequence = char(strjoin(string(sequence),''));

    % Always remove last enty (which is alsways a 1) to match node description
    sequence = sequence(1:end-1);

    for ii = length(nodeLabels):-1:1 %start from the back to ensure that the longest matching sequence is found
        node2check = nodeLabels{ii};
        node2check = node2check(~isspace(node2check)); %remove whitespace
        if endsWith(sequence,node2check)
            K = K_synth.(['v',num2str(ii)]);
            break
        end
    end
    %disp(['DEBUG:    Controller node: ',num2str(ii),', sequence: ',node2check])
    try
        u = -K * [x; u];
    catch
        error('Controller for this node not defined, something went wrong!')
    end
end
%--

ctrl_fcn = @ctrl_wrapper;
z0 = [];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [G,nodeLabels_lif] = adapt_graph(G_gen)
    nVertices_lif = size(G_gen.Nodes,1);

    nodeLabels_lif = strings(1,nVertices_lif);
    for ii = 1:nVertices_lif %save node labeling for later use in the controller
        iNodeName = G_gen.Nodes.Name{ii};
        nodeLabels_lif(ii) = iNodeName(1:end-1);
    end
    for ii = 1:nVertices_lif
        %rename nodes to numbering - this is required for the LMI descriptions
        G_gen.Nodes.Name{ii} = num2str(ii);
    end
    edges_lif = [];
    weights_lif = [];
    for ii=1:size(G_gen.Edges,1)
        %edges
        edges_lif = [edges_lif; ...
            str2double(G_gen.Edges.EndNodes{ii,1}) str2double(G_gen.Edges.EndNodes{ii,2})];
        %weights-1
        weights_lif = [weights_lif, G_gen.Edges.Weight(ii) - 1];
    end


    % Create non-lifted graph from lifted graph
    % General idea: replace each edge which corresponds to one or multiple
    % unsuccessful transmissions by concatenated edges/nodes with labels
    % [0,...0,1].
    nVertices = nVertices_lif;
    edges = edges_lif;
    weights = weights_lif;
    for iEdge=1:size(edges,1)
        if weights(iEdge) > 0 % nodes and edges have to be inserted
            nodes2insert = weights(iEdge);

            % Save start and end node. New nodes are attached to the end of the
            % node list.
            startNode = edges(iEdge,1);
            endNode = edges(iEdge,2);

            % first edge
            edges(iEdge,:) = [startNode,nVertices+1];
            weights(iEdge) = 0;
            startNode = nVertices+1;

            % in between edges
            for ii=2:nodes2insert
                edges = [edges; startNode,nVertices+ii];
                startNode = nVertices+ii;
                weights = [weights, 0];
            end

            % last edge
            edges = [edges; startNode,endNode];
            weights = [weights, 1];

            % nodes inserted
            nVertices = nVertices + nodes2insert;
        else
            % shift from 0 to 1 (mu -> alpha sequence)
            if weights(iEdge) == 0
                weights(iEdge) = 1;
            end
        end
    end

    G = digraph(edges(1:end,1)',edges(1:end,2)');
    G.Edges.Weight = weights';

end


function K = synthesize_graph_lqr_ctrl(A,B,Q,S,R,graph)
    % init
    tol = 1e-5;
    numEdges = size(graph.Edges.EndNodes,1);
    yalmip('clear');

    n = size(A,1);
    m = size(B,2);

    % create sdpvars
    for iVertex = 1:numnodes(graph)
        X.(['v',num2str(iVertex)]) = sdpvar(n,n,'symmetric'); %each node requires a separate X_i
        Y.(['v',num2str(iVertex)]) = sdpvar(m,n,'full');
    end
    sdpvar alp

    % solve initial DARE
    P = idare(A,B,Q,R,S);

    % build the LMIs
    constrLMIs = lmi('');

    % dare condition
    constrLMIs = [constrLMIs, X.('v1') - alp*inv(P) >= tol];

    for iEdge = 1:numEdges
        % vertices and label for this edge
        i = graph.Edges.EndNodes(iEdge,1);
        j = graph.Edges.EndNodes(iEdge,2);
        l = graph.Edges.Weight(iEdge); %0 or 1

        vi = ['v',num2str(i)];
        vj = ['v',num2str(j)];

        switch l
            case 0 % miss, E0
                M = [X.(vi)  , (A*X.(vi))', X.(vi)*Q;...
                     A*X.(vi), X.(vj)     , zeros(n);...
                     Q*X.(vi), zeros(n)   , Q        ];
            case 1 % hit, E1
                QSR = [Q,  S;...
                       S', R];
                XY = [X.(vi); -Y.(vi)];

                M = [X.(vi)           , (A*X.(vi)-B*Y.(vi))', XY'*QSR     ;...
                     A*X.(vi)-B*Y.(vi), X.(vj)              , zeros(n,n+m);...
                     QSR*XY           , zeros(n+m,n)        , QSR          ];
                
            otherwise
                error('Invalid edge label/mode.')
        end
        constrLMIs = [constrLMIs, M >= tol];
    end

    % Solve the LMI
    options = sdpsettings('solver','mosek','verbose',0,'debug',0);
    sol = optimize(constrLMIs,-alp,options);
    
    % obtain numerical values from sdpvars + controller
    for iVertex = 1:numnodes(graph)
        X.(['v',num2str(iVertex)]) = value(X.(['v',num2str(iVertex)]));
        Y.(['v',num2str(iVertex)]) = value(Y.(['v',num2str(iVertex)]));
        K.(['v',num2str(iVertex)]) = Y.(['v',num2str(iVertex)]) * inv(X.(['v',num2str(iVertex)]));
        %note: some of the K's will be NaN as they correspond to nodes where no controller is active and therefore no Y is part of the LMIs. Those controllers are never used, so this is ok.
    end

end