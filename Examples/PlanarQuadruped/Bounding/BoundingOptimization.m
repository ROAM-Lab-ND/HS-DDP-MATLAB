clear all
clc
% Model Hierarchy Trajectory Optimization Problem Setup
boundingModeLib = BoundingModeLib();
problem.cPhase = boundingModeLib.lib(1);
problem.nPhase = 8;
problem.nWBPhase = 8;
problem.nFBPhase = 0;
problem.phaseSeq = boundingModeLib.genModeSequence(problem.cPhase, problem.nPhase);
problem.vd = 1.5;
problem.hd = -0.13;
problem.dt = 0.001;
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
options.one_more_sweep   = 0;
%% Build Hybrid trajectory optimization problem
% Set initial condition
q0 = [0,-0.1093,-0.1542 1.0957 -2.2033 0.9742 -1.7098]';
qd0 = [0.9011 0.2756 0.7333 0.0446 0.0009 1.3219 2.7346]';
x0 = [q0;qd0];

% Create a bounding trajectory optimization problem
[phases, hybridTraj, hybridRef] = createBoundingTask(problem);

% Create reference trajectory generator
refObj = BoundingReference(hybridRef);

% Set trajectory initial condition
hybridTraj(1).Xbar{1} = x0;
hybridTraj(1).X{1} = x0;

% Generate reference
refObj.update(x0, problem);

% Build solver
hsddp = HSDDP(phases, hybridTraj);
hsddp.set_initial_condition(x0);
heuristic_bounding_controller(hybridTraj, problem); % Initialize the trajectory
[xopt, uopt, Kopt] = hsddp.solve(options);

%% Visulization
traj_G = [];
for i = 1:problem.nWBPhase
    traj_G = [traj_G, cell2mat(xopt{i})];
end
G = QuadGraphics();
G.addTrajectory(traj_G);
G.visualizeTrajectory();
