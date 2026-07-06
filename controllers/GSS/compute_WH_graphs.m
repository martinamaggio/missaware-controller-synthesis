function WHgraphs = compute_WH_graphs(con,bPlotGraphs,bPlotOnlyLiftedGraph,overrunStrategy,bSilent)
%COMPUTE WH GRAPHS
% From a given WH constraint, this function computes and returns the
% corresponding non-lifted and lifted WH graphs as MATLAB's digraph
% variable. 
%
% Requested files: createAnyWHRTgraph.m / createRowWHRTgraph.m
%
% INPUT
%   con:         A struct containing the type of constraint ('AnyHit' or 'RowHit')
%                and the respective window length 's' and number of hits/misses 'r'
%                within the windows.
%   bPlotGraphs: [optional] Boolean specifying if the resulting graphs
%                should be visualized in a plot. Default: true 
%   bPlotOnlyLiftedGraph: [optional] Boolean specifying if only the lifted
%                graph should be visualized. Default: true
%   overrunStrategy: [optional] string containing the overrun strategy,
%                used to decide whether there exist recoveries. Defaults to
%                Skip, i.e., with recoveries.
%
% OUTPUT
%   G_nonlif, G: The non-lifted/lifted WH graphs as digraph objects with
%                the fields:
%                graph.Edges: A table containing the edge definitions in the
%                             column EndNodes defined as [tail, head], where tail is where
%                             the arrow starts and the corresponding
%                             weights in the Weight column, specifying the switched system mode
%                             for each edge.
%                     .Nodes: An empty 0-by-<number of nodes> array, no
%                             content required for future handling.
%                Example: graph.Edges = EndNodes    Weight
%                                       ________    ______
%                                        1    1       1   
%                                        1    2       0   
%                                        2    1       1   
%                         represents the WHC (1,2)


if ~bSilent
    disp('    Computing WH graphs...')
end
%tic

% Check input consistency - limits to (2^15)-1
if con.s ~= int16(con.s) || con.r ~= int16(con.r)
    warning(['Either s = ',num2str(con.s),' or r = ',num2str(con.r),...
             ' is non-integer or too large for int16, rounding it...'])
end
con.r = int16(con.r);
con.s = int16(con.s);
if con.r <= 0 || con.s <= 0
    error('Invalid r or s specified.')
end

% Set defaults
if nargin < 4
    bKill = false;
else
    if ismember(overrunStrategy,{'Kill','kill'})
        bKill = true;
    else
        bKill = false;
    end
end
if nargin < 3
    bPlotOnlyLiftedGraph = true;
end
if nargin < 2
    bPlotGraphs = true;
end


%% Obtain graph
if ~bSilent
    disp(['        computing lifted graph for ',con.type,'(',num2str(con.r),',',num2str(con.s),')'])
end
switch con.type
    case 'AnyHit'
        G_gen = create_any_WH_graph(con.r,con.s);
    case 'RowHit'
        G_gen = create_row_WH_graph(con.r,con.s);
    otherwise
        error(['No WH constraint type ',con.type,' defined.'])
end


%% Reformulate it to graph interface with numbered edges and nodes
% Edge labels stay in 0/1 notation for hits/misses, and do not use H/M/R
% symbols. Those are only used for plotting the non-lifted graph.
if ~bSilent
    disp('        adapting to MATLAB interface...')
end
% instead of sequence string names

nVertices_lif = size(G_gen.Nodes,1);

nodeLabels_lif = strings(1,nVertices_lif);
for ii = 1:nVertices_lif %save node labeling for later use in the controller
    iNodeName = G_gen.Nodes.Name{ii};
    if bKill %Kill: rm last entry to match with graph definition in paper
        nodeLabels_lif(ii) = iNodeName(1:end-1);
    else %Skip: rm first entry
        nodeLabels_lif(ii) = iNodeName(2:end);
    end
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


%% Create non-lifted graph from lifted graph
if ~bSilent
    disp('        computing non-lifted graph...')
end
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

% Plot graphs
if bPlotGraphs
    if ~bSilent
        disp('        plotting graph(s)...')
    end
    if bPlotOnlyLiftedGraph
        figure
        fig_lif.handler = subplot(1,1,1);
        fig_lif.plotHandler = plot_graph(edges_lif,weights_lif,true,true,bKill);

        fig_nonLif = plot([],[],'-'); % create empty handler to avoid errors
    else
        figure
        fig_lif.handler = subplot(1,2,2);
        fig_lif.plotHandler = plot_graph(edges_lif,weights_lif,true,true,bKill);

        fig_nonLif.handler = subplot(1,2,1);
        fig_nonLif.plotHandler = plot_graph(edges,weights,true,false,bKill);
    end
else
    fig_lif = plot([],[],'-'); % create empty handlers to avoid errors
    fig_nonLif = plot([],[],'-');
end

if ~bSilent
    disp('        finishing up...')
end

% Create graph objects
G = digraph(edges(1:end,1)',edges(1:end,2)');
G.Edges.Weight = weights';

G_lif = digraph(edges_lif(1:end,1)',edges_lif(1:end,2)');
G_lif.Edges.Weight = weights_lif';

% Combine to constraint struct
WHgraphs.graph_nonlif = G;
WHgraphs.graph = G_lif;
WHgraphs.fig_nonLif = fig_nonLif;
WHgraphs.fig = fig_lif;
WHgraphs.nodeLabels_lif = nodeLabels_lif; %save node labeling for later use in the controller

%disp(['    Computing WH graphs finished. Elapsed time: ',num2str(toc),' seconds.'])

end


function h = plot_graph(e,w,bReplaceEdgeLabels,bIsLifted,bKill)
    % Internally, the digraph object uses the 0 and 1 notation for hits and misses
    s = e(1:end,1)';
    t = e(1:end,2)';
    G = digraph(s,t);

    % If no edge labeling is required, just plot and return
    if ~bReplaceEdgeLabels
        G.Edges.Weight = w';
        h = plot(G,'EdgeLabel',G.Edges.Weight,'Layout','layered');
        if bIsLifted
            title('WH graph')
        else
            title('Non-lifted WH graph')
        end
        subtitle([num2str(1),' = hit, ',num2str(0),' = miss'])
        return
    end

    % Replacing the labels depending on what type of graph we have
    h = plot(G,'Layout','layered');

    for iEdge=1:length(w)
        if bIsLifted
            % Lifted graph, replace according to the alpha-strategy
            if w(iEdge) == 0
                % alpha = 0 => hit
                labeledge(h,iEdge,[num2str(0),' (H)']);
            else
                % alpha > 0 => misses followed by a hit or recovery
                if bKill
                    labeledge(h,iEdge,[num2str(w(iEdge)),' (HM^',num2str(w(iEdge)),')']);
                else
                    labeledge(h,iEdge,[num2str(w(iEdge)),' (M^',num2str(w(iEdge)),'R',')']);
                end
            end

        else
            % Non-lifted graph, replace edge labels from 0/1 to H/M or H/M/R, depending on the strategy
            if w(iEdge) == 0
                % miss
                labeledge(h,iEdge,'M');
            else
                % hit or recovery
                if bKill
                    labeledge(h,iEdge,'H'); %only hit, no recovery exists
                else
                    inEdges = inedges(G,s(iEdge)); %find all incoming edges of the start node
                    if any(w(inEdges) == 0)
                        % recovery
                        labeledge(h,iEdge,'R');
                    else
                        % hit
                        labeledge(h,iEdge,'H');
                    end
                end
            end
        end
    end
    if bIsLifted
        title('WH graph')
    else
        title('Non-lifted WH graph')
    end
    if bKill
        subtitle('H = hit, M = miss')
    else
        subtitle('H = hit, M = miss, R = recovery')
    end

end
