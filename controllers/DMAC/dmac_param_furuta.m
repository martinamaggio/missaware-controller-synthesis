% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c] = dmac_param_furuta()
    % Parameters used for the Furuta example controller design 
    
    % System dynamics
    [A_c,B_c] = furuta_matrices();
end