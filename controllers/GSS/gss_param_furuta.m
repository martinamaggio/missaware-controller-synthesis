function system = gss_param_furuta(period,actuator_strategy,overrun_strategy)
    % Parameters used for the Furuta example controller design 
    
    % System dynamics
    [A_c,B_c] = furuta_matrices();
    ss_c = ss(A_c,B_c,eye(3),zeros(3,1));
    ss_d = c2d(ss_c,period);
    system.A = ss_d.A;
    system.B = ss_d.B;
    
    % system.A = [1.0026   0.0050  0;...
    %             1.0399   1.0026  0;...
    %             -0.0675 -0.0002  1];
    % system.B = [-0.0843; -33.7508; 39.2131];
    
    % Controller
    K_original = [0.3750 0.0250 0.0125];
    K_lqr      = [0.4280 0.0307 0.0119];
    system.K = K_original;
    
    % Performance matrices
    system.C  = [1 0.1 0.1];
    system.D  = 10;
    system.Bw = [0; 0; 1];
    system.Dw = 0;
end