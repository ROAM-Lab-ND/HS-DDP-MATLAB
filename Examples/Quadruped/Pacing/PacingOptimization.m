clear
clc
% Set options of HSDDP solver
options.alpha            = 0.1;        % linear search update param
options.gamma            = 0.01;       % scale the expected cost reduction
options.beta_penalty     = 8;          % penalty update param
options.beta_relax       = 0.1;        % relaxation update param
options.beta_reg         = 2;          % regularization update param
options.beta_ReB         = 7;
options.max_DDP_iter     = 5;          % maximum DDP iterations
options.max_AL_iter      = 4;          % maximum AL iterations
options.DDP_thresh       = 0.001;      % Inner loop opt convergence threshold
options.AL_thresh        = 1e-3;       % Outer loop opt convergence threshold
options.tconstraint_active        = 1;          % Augmented Lagrangian active
options.pconstraint_active       = 1;          % Reduced barrier active
options.feedback_active  = 1;          
options.smooth_active    = 0;          % Smoothness active
options.parCalc_active   = 1;          % compute cost and dynamics partials in forward sweep (default)
options.Debug            = 1;          % Debug active

[phases, hybridT, hybridRef] = createPacingTask();

% set initial condition
pos = zeros(3,1);
rpy = zeros(3,1);
qa = zeros(12,1);
vel = zeros(3,1);
rpyrate = zeros(3,1);
qad = zeros(12,1);
q = [pos; rpy; qa];
qd = [vel; rpyrate; qad];
x = [q; qd];

hsddp = HSDDP(phases, hybridT);
[xopt, uopt, Kopt] = hsddp.solve(options);