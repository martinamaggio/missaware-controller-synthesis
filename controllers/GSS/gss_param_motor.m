function system = gss_param_motor(period,actuator_strategy,overrun_strategy)
    % Parameters used for the motor example controller design 
    
    % System dynamics
    [A_c,B_c] = motor_matrices();
    nx = size(A_c,1);
    nu = size(B_c,2);
    ss_c = ss(A_c,B_c,eye(nx),zeros(nx,nu));
    ss_d = c2d(ss_c,period);
    system.A = ss_d.A;
    system.B = ss_d.B;
        
    % Controller
    system.K = [];
    
    % Performance matrices
    switch actuator_strategy
        case 'zero'
%             system.C  = [0.8 0.8 0.2];
%             system.D  = [0.1 0.1];
%             system.Bw = [0; 0; 1];
%             system.Dw = 0;

            system.C  = [30 10 0];
            system.D  = [0.0 0.0];
            system.Bw = [10; 10; 10];
            system.Dw = 0;
        case 'hold'
            switch overrun_strategy
                case 'kill'
                    system.C  = [1.0 1.0 0.4];
                    system.D  = [0.1 0.1];
                    system.Bw = [0; 0; 10];
                    system.Dw = 0;
                case 'skip'
                    system.C  = [2 2 0.7];
                    system.D  = [0.1 0.1];
                    system.Bw = [0; 0; 10];
                    system.Dw = 0;
                otherwise
                    error('Overrun strategy not valid')
            end
        otherwise
            error('Actuator strategy not valid')
    end
end