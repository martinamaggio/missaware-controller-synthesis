function L = robust_feedback(A,B,Qc,h,holdmat,actuator_strategy)

% holdmat = [pvec hvec] describes the possible HOLD intervals (in multiples of h)

% Sanity checks
if nargin < 5
	error('To few arguments to function: robust_feedback(A,B,Qc,h,holdmat)');
end

if norm(sum(holdmat(:,1))-1) > 1e-6
	sum(holdmat(:,1))
	error('Probabilities in pvec do not add up to 1')
end

nh = size(holdmat,1);  % number of possible h = number of rows

nx = size(A,1);
nu = size(B,2);

% Sample the system for all different hold intervals
for k = 1:nh
	[Phie,~,Qe] = calcc2d([A B; zeros(nu,nx+nu)],zeros(nx+nu),Qc,h*holdmat(k,2));
	Phi{k} = Phie(1:nx,1:nx);
	Gam{k} = Phie(1:nx,nx+1:nx+nu);
	Q1{k} = Qe(1:nx,1:nx);
	Q2{k} = Qe(nx+1:nx+nu,nx+1:nx+nu);
	Q12{k} = Qe(1:nx,nx+1:nx+nu);
end

if (strcmp(actuator_strategy,'zero'))
    for k = 2:nh
        Gam{k} = Gam{1}; % From Pazzaglia PhD Thesis (2020)
    end
end

% Solve stochastic Riccati equation iteratively
S = zeros(size(Phi{1},1));
Snew = eye(size(S,1));
delta = 0.1;
power = -5;
while norm(S-Snew) > 1e-9
	S = Snew;
	X = zeros(size(S)+nu);
	for k = 1:size(holdmat,1)
		X = X + holdmat(k,1) * ([Phi{k} Gam{k}]'*S*[Phi{k} Gam{k}] + [Q1{k} Q12{k}; Q12{k}' Q2{k}]);
	end
	L = X(end-nu+1:end,end-nu+1:end) \ X(end-nu+1:end,1:end-nu);
	Snew = X(1:end-nu,1:end-nu) - L'*X(end-nu+1:end,end-nu+1:end)*L;
	if norm(Snew) > 1e10
        % modified hack
        S = zeros(size(Phi{1},1));
        Snew = eye(size(S,1))*delta^power;
    	power = power + 1;
        if (power == 10)
            error('S problem!')
        end
    end
end
end

