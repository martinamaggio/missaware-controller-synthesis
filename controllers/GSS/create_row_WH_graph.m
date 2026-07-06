function G = create_row_WH_graph(n,m)
%CREATE ROW WH GRAPH
% Generates the WH graph G for the row weakly-hard constraint
% (type: <n,m>).
% WH constraint: In any window of m consecutive transmissions,
% there are at least n consecutive transmissions that are successful.
%
% INPUT
%   n,m:    Numeric parameters specifying the row WH constraint
%
% OUTPUT
%   G:      The graph as a MATLAB graph object
%
% Code adapted from https://codeocean.com/capsule/5102040/tree/v1


%% initialize an empty directed graph
G = digraph;

%% create initial node 
%assume n consecutive successes - corresponds to worst case for future transmissions since it induces no knowledge
current_vector = ones(1,n);
G = addnode(G,{num2str(current_vector)});

%% initialize loop variable to go through all nodes
current_node = 1;

while current_node <=  size(G.Nodes,1)
    
    %read out label of current_node
    current_vector = str2num(G.Nodes.Name{current_node});
    
    %reset indicator for finishing the loop and initialize ix_label 
    is_finished = 0;
    ix_label = 1;
    
    %loop over labels (corresponds to number of losses after successful
    %transmissions -1)
    while ix_label <= (m-n+1) && is_finished == 0  
        
        %add (ix_label-1) 0's and a final 1 to the current node label
        current_vector_new = [current_vector,zeros(1,ix_label-1),1];
        
        %check whether constraint is satisfied (this has to be done for a
        %vector with (n-2) added 1's at the end to avoid FINITE sequences
        %that satisfy the constraint but that can not be part of an
        %INFINITE sequence that satisfies the constraint (n-2 since one 
        %already has one 1 at the end and over n 1's it is trivially satisfied)
        current_vector_check = [current_vector_new, ones(1,n-2)];
        is_satisfied = 1;
        for ix_check = 1:max(length(current_vector_check)-m+1,1)
            %compute max number of consec 1's in current_vector_check over
            %each window of length m, starting from ix_check
            vec_aux = current_vector_check(ix_check:min(length(current_vector_check),ix_check+m-1));
            level_aux = find(diff([0,vec_aux,0]));
            rise_aux = level_aux(1:2:end-1);
            fall_aux = level_aux(2:2:end);
            succ_aux = fall_aux-rise_aux;
            if max(succ_aux) < n
                is_satisfied = 0;
            end
        end
        
        %if constraint is not satisfied the loop can be finished
        if is_satisfied == 0
            is_finished = 1;
        else %otherwise an edge has to be created to an existing node or to a node that has to be created
            
            %shorten current_vector_new to relevant part (until last consecutive m 1's)
            ix_cut = max(strfind(current_vector_new,ones(1,n)));
            current_vector_new = current_vector_new(ix_cut:end);
            
            %check whether node with this label exists
            new_node = findnode(G,{num2str(current_vector_new)});
            if new_node ~= 0
                % if yes, add edge (with label: ix_label)
                G = addedge(G,current_node,new_node,ix_label);
            else
                % if no, add node and consequently edge (with label:
                % ix_label)
                G = addnode(G,{num2str(current_vector_new)});
                new_node = findnode(G,{num2str(current_vector_new)});
                G = addedge(G,current_node,new_node,ix_label);
            end
        end
        % go to next label (more losses)
        ix_label = ix_label+1;
    end
    
    % go to next node
    current_node=current_node+1;
end

end