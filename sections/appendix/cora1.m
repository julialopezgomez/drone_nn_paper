% Closed-loop nonlinear system (PID embedded in dynamics)
nX = 13;   % 10 plant states + 3 PID integral states
nU = 1;    % no external inputs (closed-loop)
fCL   = @(x,u) quadPayloadPID_ode_lin(x,u,p);
sysCL = nonlinearSys(fCL, nX, nU);

% Initial condition with bounded uncertainty
x0_nom  = [0; 5; 0; 0; 0; 0; 1; 0; 0; 0];
e_int0  = [0; 0; 0];
x0_full = [x0_nom; e_int0];

eps0 = 0.01;
params.R0 = zonotope(interval(x0_full - eps0, x0_full + eps0));

% Reachability parameters
params.tFinal = 4;
params.U      = zonotope(0);

options.timeStep      = 0.001;
options.taylorTerms   = 4;
options.zonotopeOrder = 20;
options.alg           = 'poly';
options.tensorOrder   = 3;
options.errorOrder    = 3;
options.intermediateOrder = 3;

% Specification sets
INF = 1e6;
lb_goal = -INF*ones(nX,1);  ub_goal = INF*ones(nX,1);
lb_goal(2) = 2.0;           ub_goal(2) = 10.0;
goalSet = interval(lb_goal, ub_goal);

lb_avoid = -INF*ones(nX,1); ub_avoid = INF*ones(nX,1);
lb_avoid(2) = 0.0;          ub_avoid(2) = 1.0;
avoidSet = interval(lb_avoid, ub_avoid);

% Time-bounded specifications
spec1 = specification(goalSet,  'safeSet',   interval(params.tFinal-1, params.tFinal));
spec2 = specification(avoidSet, 'unsafeSet', interval(0, params.tFinal));
spec  = add(spec1, spec2);

% Reachability verification
[RCL, resCL] = reach(sysCL, params, options, spec);