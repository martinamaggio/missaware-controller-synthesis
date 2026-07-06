function [ctrl_fcn, z0] = setup_acc_control(psys,prt)
    
% Notes
% - restriction: only Kill and Hold
% - only anyHit with the specific periodic hit/miss pattern!
% - implemented is Corollary 1

% extract the needed system and real-time parameters
plant_name = psys.plant_name;
period = psys.period;
kap = prt.w; mu = prt.h;
strategy_overrun = prt.overrun_strategy;
strategy_actuator = prt.actuator_strategy;


if ~(strategy_overrun=="kill")
    error('AccControl controller assumes a kill overrun strategy.')
end
if ~(strategy_actuator=="hold")
    error('AccControl controller assumes the hold actuator strategy.')
end


%% Define system - Furuta pendulum
% A = [1.0026   0.0050  0;...
%             1.0399   1.0026  0;...
%             -0.0675 -0.0002  1];
% B = [-0.0843; -33.7508; 39.2131];

% nominal system dynamics 
[A,B,Q1,Q12,Q2] = feval(['acccontrol_param_',plant_name],period);
n_orig = size(A,1);
m_orig = size(B,2);
A = [A, B; zeros(m_orig,n_orig+m_orig)];
B = [zeros(n_orig,m_orig); eye(m_orig)];

n = size(A,1);
m = size(B,2);

Q = [Q1, Q12; Q12', Q2];


%% Synthesize controllers
for d_gamma_iter = 1:kap
    A_tilde = A^(d_gamma_iter);
    B_tilde = zeros(size(B));
    for ii = 0:d_gamma_iter-1
        B_tilde = B_tilde + A^ii*B;
    end

    Q_tilde = Q;
    if d_gamma_iter > 1
        for ii = 1:d_gamma_iter-1
            sumAB = zeros(size(B));
            for jj = 0:ii-1
                sumAB = sumAB + A^jj*B;
            end
            Phi = [A^ii, sumAB; zeros(m,n), eye(m)];
            Q_tilde = Q_tilde + Phi' * Q * Phi;
        end
    end

    Q1_tilde = Q_tilde(1:n,1:n);
    Q12_tilde = Q_tilde(1:n,n+1:end);
    Q2_tilde = Q_tilde(n+1:end,n+1:end);

    %rank([B_tilde, A_tilde*B_tilde, A_tilde^2*B_tilde]) %debug: check controllability

    S = idare(A_tilde,B_tilde,Q1_tilde,Q2_tilde,Q12_tilde);
    K.(['d',num2str(d_gamma_iter)]) = inv(Q2_tilde * (B_tilde' * S * B_tilde)) * (B_tilde' * S * A_tilde + Q12_tilde');
end


%--
function [u,z] = ctrl_wrapper(x,z,u,~,k)
    % Find next mandatory job at instance km
    flag = true;
    km = k + 1; 
    while flag
        if km == floor(ceil(km*mu/kap)*kap/mu)
            flag = false;
        else
            km = km + 1;
        end
    end
    d_gamma = km - k; %distance to next mandatory job

    % controller
    try
        u = -K.(['d',num2str(d_gamma)]) * [x;u];
    catch
        error('Controller for this node not defined, something went wrong!')
    end
end
%--

ctrl_fcn = @ctrl_wrapper;
z0 = [];

end
