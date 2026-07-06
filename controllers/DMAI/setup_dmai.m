function [ctrl, z0] = setup_dmai(psys,prt,baseline_ctrl)
    %%
    % Deadline-Miss-Adaptive Controller Imlpementation (DMAI):
    % implements the adaptation of the controller matrices based on the number
    % of deadline misses since last execution of the controller.
    % This controller implementation is (1) independent of the actuator
    % strategy and (2) assumes the kill strategy for the jobs missing the
    % deadline.
    % 
    % Inputs
    %   psys            : struct containing the system and simulation parameters
    %   prt             : struct of the real-time setting parameters
    %   baseline_ctrl   : baseline dynamic controller

    % extract the needed system and real-time parameters
    C = psys.C;
    sampling_period = psys.period;
    strategy_overrun = prt.overrun_strategy;

    if ~(strategy_overrun=='kill')
        error('DMAI controller assumes a kill overrun strategy')
    end

    % baseline controller design
    % F     : state-to-state matrix of controller
    % G     : input-to-state matrix of controller
    % H     : state-to-output matrix of controller
    % K     : input-to-output matrix of controller
    [F,G,H,K] = baseline_ctrl(sampling_period,C);

    % store the measurement from last hit
    y_old = zeros(size(C,1),1);

    function [u,z] = ctrl_dmai(x,z,~,sequence,~)
        % Inputs
        % z     : state of controller
        % y     : new measurement
        % y_old : measurement from last hit
        % q     : number of consecutive deadline misses since last hit
        % Outputs
        % u_kq1 : control action
        % z_kq1 : controller state

        y = C*x; % actual output measurement
        q = miss_since_hit(sequence);

        u = H_z(F, H, q)*z + H_y(F, G, H, q)*y_old + K_y(F, G, H, K, q)*y;
        z = F_z(F, q)*z    + F_y(F, G, q)*y_old    + G_y(F, G, q)*y;

        y_old = y;
    end

    ctrl = @ctrl_dmai;
    z0 = zeros(size(F,2),1);
end

function m = miss_since_hit(sequence)
    m=0;
    for i = length(sequence)-1:-1:1
        if sequence(i)==1
            return
        end
        m = m + 1;
    end
end

function mat = F_z(F, q)
    mat = F^(q+1);
end

function mat = F_y(F, G, q)
    mat = zeros(size(F*G));
    for i = 0:q
        mat = mat + (i/(q+1))*F^i*G;
    end
end

function mat = G_y(F, G, q)
    mat = zeros(size(F*G));
    for i = 0:q
        mat = mat + ((q+1-i)/(q+1))*F^i*G;
    end
end

function mat = H_z(F, H, q)
    mat = H*F^(q);
end

function mat = H_y(F, G, H, q)
    mat = zeros(size(H*F*G));
    for i = 1:q
        mat = mat + H*(i/(q+1))*F^(i-1)*G;
    end
end

function mat = K_y(F, G, H, K, q)
    mat = K;
    for i = 1:q
        mat = mat + H*((q+1-i)/(q+1))*F^(i-1)*G;
    end
end