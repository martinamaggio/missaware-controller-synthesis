function [A,B,Q1,Q12,Q2] = acccontrol_param_furuta(period)
    % Parameters used for the Furuta example controller design 
    
    % System dynamics
    [A_c,B_c] = furuta_matrices();
    nx = size(A_c,1);
    nu = size(B_c,2);
    ss_c = ss(A_c,B_c,eye(nx),zeros(nx,nu));
    ss_d = c2d(ss_c,period);
    A = ss_d.A;
    B = ss_d.B;
    
    Q1 = eye(nx+nu); %pos. def!
    Q12 = zeros(nx+nu,nu);
    Q2 = 1*eye(nu); %pos. def!
end