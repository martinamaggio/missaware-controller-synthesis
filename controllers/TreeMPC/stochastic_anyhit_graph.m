% STOCHASTIC_ANYHIT_GRAPH generates a list of nodes of a minimal graph representing the
% anyHit(m,k) constraint and a transition probability matrix
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [nodes,tpm] = stochastic_anyhit_graph(m,k,p_miss)
% Generates a list of nodes of a minimal graph representing the
% anyHit(m,k) constraint and a transition probability matrix for the
% edges (can be used with 'kill' and 'skip' overrun strategy).
%
% Inputs
%   m, k    : parameters of anyHit(m,k) constraint
%   p_miss  : probability of the next consecutive deadline miss
%
% Outputs
%   nodes   : list of nodes (labels are char arrays of hit/miss sequence)
%   tpm     : transition probability matrix
    
    n_miss_max = k-m;
    [nodes,tpm] = stochastic_anymiss_graph(n_miss_max,k,p_miss);
end