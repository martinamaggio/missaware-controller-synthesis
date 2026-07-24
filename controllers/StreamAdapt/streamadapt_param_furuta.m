% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function [A_c,B_c,C,design_func] = streamadapt_param_furuta(C)
    % Parameters and nominal control design function used for the Furuta example controller design 
    
    % poles used for controller and observer design
    cl_poles = [-20, -30+0.5i, -30-0.5i, -100];
    obs_poles = [-80, -150+2i, -150-2i, -500];

    [A_c,B_c] = furuta_matrices();

    function [K,L] = nominal_ctrl_design(A,B,C,period)
        K = place(A,B,exp(cl_poles.*period));
        L = place(A',C',exp(obs_poles.*period))';
    end

    design_func = @nominal_ctrl_design;

end