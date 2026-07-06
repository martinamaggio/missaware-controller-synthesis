function [A_aug,B_aug,Q,S,R] = graphlqr_param_motor(period)
    % Parameters used for the motor example controller design 
        
    [A_c,B_c] = motor_matrices();
    nx = size(A_c,1);
    nu = size(B_c,2);
    ss_c = ss(A_c,B_c,eye(nx),zeros(nx,nu));
    ss_d = c2d(ss_c,period);
    A = ss_d.A;
    B = ss_d.B;
    
    A_aug = [A B; zeros(nu,nx+nu)];
    B_aug = [zeros(nx,nu); eye(nu)];
    
    % weighting matrices (includes augmented state) - can be tuned
    Q = diag([1,1,1,0.1,0.1]); %pos. def!
    S = zeros(nx+nu,nu);
    R = 1*eye(nu); %pos. def!
end