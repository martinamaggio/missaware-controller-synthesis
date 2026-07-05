function [F,G,H,K] = ctrl_furuta_linear(sampling_period,C)
    % This function designs a linear dynamic controller for the linear model
    % of the furuta pendulum.

    [A,B] = furuta_matrices();

    cl_poles = [-10, -30+0.5i, -30-0.5i];
    obs_poles = 4*cl_poles;


    fb_gain = place(A,B,cl_poles);
    obs_gain = place(A',C',obs_poles)';

    % controller in state-space form
    A_K = A - B*fb_gain - obs_gain*C;
    B_K = obs_gain;
    C_K = -fb_gain;
    D_K = zeros(1, size(C,1));
    % move to discrete time
    K_ss = ss(A_K, B_K, C_K, D_K);
    K_ss_d = c2d(K_ss, sampling_period, 'zoh');
    F = K_ss_d.A;
    G = K_ss_d.B;
    H = K_ss_d.C;
    K = K_ss_d.D;
end