clear all
clc
problem.dt = 0.001;
problem.len_horizons = [544, 457];

% Set options of HSDDP solver
options.alpha            = 0.1;        % linear search update param
options.gamma            = 0.01;       % scale the expected cost reduction
options.beta_penalty     = 2;          % penalty update param
options.beta_relax       = 0.1;        % relaxation update param
options.beta_reg         = 2;          % regularization update param
options.beta_ReB         = 7;
options.max_DDP_iter     = 1;          % maximum DDP iterations
options.max_AL_iter      = 1;          % maximum AL iterations
options.DDP_thresh       = 0.001;      % Inner loop opt convergence threshold
options.AL_thresh        = 1e-3;       % Outer loop opt convergence threshold
options.tconstraint_active        = 1;          % Augmented Lagrangian active
options.pconstraint_active       = 0;          % Reduced barrier active
options.feedback_active  = 1;          
options.Debug            = 1;          % Debug active

x0 = [4, 0]';
[phases, hybridT] = createBouncingBallTask(problem);
hybridT(1).Xbar{1} = x0;
hybirdT(1).X{1} = x0;

% Initialize the trajectory using the Nathon's data
% Initial guss would be set to zero if not initialized
initializeBouncingTraj(hybridT);

hsddp = HSDDP(phases, hybridT);
hsddp.set_initial_condition(x0);
[xopt, uopt, Kopt, Info] = hsddp.solve(options);
hsddp_data.xopt = xopt;
hsddp_data.uopt = uopt;
hsddp_data.Kopt = Kopt;
hsddp_data.dt = problem.dt;
compare_hsddp2saltation(hsddp_data);
plot_constraint_violation(Info.max_violations);

