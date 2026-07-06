function ctrl = robust_lqctrl(A,B,Qc,h,holdmat,predmat,overrun_strategy,actuator_strategy)

% holdmat = [pvec hvec] describes the different possible hold intervals.
% Each row contains a probability and an interval length (in multiples of h)

% predmat = [pvec Gam2vec Gam1vec] describes the different possible combinations
% of held (old) control signals until the currently calculated feedback is
% applied. Each row contains a probability and two interval lengths (in
% multiples of h). Gam2 is always zero except for under the Queue(1) strategy.

% Calculate robust state feedback gain based on hold interval probabilities
if norm(sum(holdmat(:,1))-1) > 1e-6
	error('Probabilities in holdmat do not add up to 1')
end
L = robust_feedback(A,B,Qc,h,holdmat,actuator_strategy);

% Calculate average prediction matrices based on IO delay probabilities
if norm(sum(predmat(:,1))-1) > 1e-6
	error('Probabilities in predmat do not add up to 1')
end

sys = ss(A,B,eye(size(A)),0);
nx = size(A,1);
nu = size(B,2);
Gam2predbar = 0;
Gam1predbar = 0;
Phipredbar = 0;
for k = 1:size(predmat,1)
	hk = h * sum(predmat(k,2:3));  % Total prediction interval
	tau = h * predmat(k,2);        % Part belonging to Gam2
	sys.inputdelay = tau;%max(tau,sqrt(eps)*h);
	[Phie,Game] = ssdata(absorbDelay(c2d(sys,hk)));
	Gam2predk = Phie(1:nx,nx+1:end);
    if (k > 1 && strcmp(overrun_strategy,'kill') && strcmp(actuator_strategy,'zero')) 
        Gam1predk = 0*Game(1:nx,:); % Case where control input is 0 during current iteration
    elseif (k > 1 && strcmp(overrun_strategy,'skip') && strcmp(actuator_strategy,'zero'))
        % Gam1predk = Gam1predk; % Case where old control input is active
                                 % only for one period, then set to 0
    else
	    Gam1predk = Game(1:nx,:);
    end
	Phipredk = Phie(1:nx,1:nx);
	Gam2predbar = Gam2predbar + predmat(k,1) * Gam2predk;
	Gam1predbar = Gam1predbar + predmat(k,1) * Gam1predk;
	Phipredbar = Phipredbar + predmat(k,1) * Phipredk;
end

if isempty(Gam2predbar)
    Gam2predbar = zeros(size(Gam1predbar));
end

% Formulate controller state-space equations
Areg = [-L*Gam1predbar -L*Gam2predbar; eye(nu) zeros(nu)];
Breg = [-L*Phipredbar; zeros(nu,nx)];
Creg = [-L*Gam1predbar -L*Gam2predbar];
Dreg = -L*Phipredbar;

ctrl = ss(Areg,Breg,Creg,Dreg,-1);

