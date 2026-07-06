function [A_aug,B_aug,Q,S,R] = graphlqr_param_furuta(period)
    % Parameters used for the Furuta example controller design 
    
    % A = [1.0026   0.0050  0;...
    %             1.0399   1.0026  0;...
    %             -0.0675 -0.0002  1];
    % B = [-0.0843; -33.7508; 39.2131];
    
    [A_c,B_c] = furuta_matrices();
    ss_c = ss(A_c,B_c,eye(3),zeros(3,1));
    ss_d = c2d(ss_c,period);
    A = ss_d.A;
    B = ss_d.B;
    
    A_aug = [A B; zeros(1,4)];
    B_aug = [zeros(3,1); 1];
    
    % weighting matrices - can be tuned
    Q = diag([1,1,1,0.1]); %pos. def!
    S = zeros(4,1);
    R = 1; %pos. def!
end