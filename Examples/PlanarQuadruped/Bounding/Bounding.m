%% Generate support functions
clear all
clc
% addpath(genpath('../..'));
addpath(genpath(pwd));
rmpath('Backup', 'Prep');
dt = 0.001;
FIRSTRUN = 0;
 
% build whole-body model for planar mc
WBMC2D = PlanarQuadruped(dt);
build2DminiCheetah(WBMC2D);

% build floating-base model for planar mc
FBMC2D = PlanarFloatingBase(dt);

% If this is the first time, run support functions to generate neccessary
% functions
if FIRSTRUN
    WBDynamics_support(WBMC2D);
    FBDynamimcs_support();
    WB_terminal_constr_support(WBMC2D);
end

%% Define a bounding gait
% One bounding gait cycle has 4 phases (1->BS,2->FL1,3->FS,4->FL2)
bounding = Gait('bounding');
bounding.setBasicTimings([0.08,0.1,0.08,0.1]);
currentPhase = 1; 

%% Set problem data and optimization options
problem_data.n_Phases       = 8;       % total nuber of phases
problem_data.n_WBPhases     = 2;       % number of whole body phases
problem_data.n_FBPhases     = 6;       % number of floating-base phases
problem_data.phaseSeq       = bounding.get_gaitSeq(currentPhase, problem_data.n_Phases+1);
problem_data.dt             = dt;
problem_data.t_horizons     = bounding.get_timeSeq(currentPhase, problem_data.n_Phases);
problem_data.N_horizons     = floor(problem_data.t_horizons./problem_data.dt);
problem_data.ctrl_horizon   = problem_data.N_horizons(1);
problem_data.vd             = 1.5;     % desired forward speed m/s

options.alpha            = 0.1;        % linear search update param
options.gamma            = 0.01;       % scale the expected cost reduction
options.beta_penalty     = 8;          % penalty update param
options.beta_relax       = 0.1;        % relaxation update param
options.beta_reg         = 2;          % regularization update param
options.beta_ReB         = 7;
options.max_DDP_iter     = 3;          % maximum DDP iterations
options.max_AL_iter      = 3;          % maximum AL iterations
options.DDP_thresh       = 0.001;      % Inner loop opt convergence threshold
options.AL_thresh        = 1e-3;       % Outer loop opt convergence threshold
options.AL_active        = 1;          % Augmented Lagrangian active
options.ReB_active       = 1;          % Reduced barrier active
options.feedback_active  = 1;          
options.smooth_active    = 0;          % Smoothness active
options.parCalc_active   = 1;          % compute cost and dynamics partials in forward sweep (default)
options.Debug            = 1;          % Debug active
options.gap_active       = 0;          % gap active   
                            
%% Run HSDDP
% Initial condition
q0 = [0,-0.1093,-0.1542 1.0957 -2.2033 0.9742 -1.7098]';
qd0 = [0.9011 0.2756 0.7333 0.0446 0.0009 1.3219 2.7346]';
x0 = [q0;qd0];

% Initlialize mhpcController
controller = mhpcController(WBMC2D, FBMC2D, bounding, problem_data, options);

% create simulator using planar miniCheetah model and ground at -0.404
sim = Simulator(WBMC2D);
sim.set_groundInfo(-0.404);
% Set controller to the simulator
sim.set_Controller(controller);

% Initialize delays for last phase and current phase
% Last delay tells simulator and controller to extract some time period
% from the current phase.
lastDelay = 0;
currentDelay = 0;
x0_opt = x0;
x0_sim = x0;
t0 = 0;
X = [];
totalCost = 0;
J = 0;

% Disturbance information
% Disturbance start at 30th time step and ends at 60th time step
disturbInfo.start = 30;
disturbInfo.end = 60;
disturbInfo.active = 0;
disturbInfo.magnitude = 0;

maxMPCIter = 16;
% Preallocate memory for simulated trajectory information
simTrajectory = repmat(struct('X', zeros(WBMC2D.xsize, 100), ...
                              'U', zeros(WBMC2D.usize, 100), ...
                              'Y', zeros(WBMC2D.ysize, 100), ...
                              't', zeros(1, 100), ...
                              'Xopt', zeros(WBMC2D.xsize, 100), ...
                              'Uopt', zeros(WBMC2D.usize, 100), ...
                              'Kopt', zeros(WBMC2D.usize, WBMC2D.xsize, 100)), [1, maxMPCIter]);

for i = 1:maxMPCIter
    controller.runHSDDP(x0_opt, options);
    
    % Tell the controller the delay of last phase
    controller.InformControllerDelay(lastDelay);       
    
    % Intialize simulator with scheduled phase sequence, phase horizons and
    % control horizons (to be applied)
    sim.set_horizonParams(problem_data.phaseSeq, problem_data.N_horizons,problem_data.N_horizons(1));
    
    % Recalculate the phase horizon considering the effect of last delay
    sim.recalcHorizonforDelay(lastDelay);
    
    % Activate disturbance at second flight
    if i == 4
        disturbInfo.active = 1;
    end
    
    % Run simulator
    % predidx indicates when delay = 0. This should be used as the initial
    % condition for the next MHPC planning.    
    [J, X, U, Y, time, predidx, collision] = sim.run(x0_sim, t0, currentDelay, disturbInfo);
    totalCost = totalCost + J;
    
    simTrajectory(i).X = X(:,1:end-1);
    simTrajectory(i).U = U;
    simTrajectory(i).Y = Y;
    simTrajectory(i).t = time;
    simTrajectory(i).Xopt = controller.xopt;
    simTrajectory(i).Uopt = controller.uopt;
    simTrajectory(i).Kopt = controller.Kopt;
    
    t0 = time(end);
    
    if collision == 1        
        fprintf('Simulation stops because of collision. /n');
        break;
    end
    
    x0_opt = X(:,predidx); % Initial condition for next optimization
    
    x0_sim = X(:,end);     % Initial condition for next simulation
    
    lastDelay = currentDelay;  % Update delay
    
    % Update problem data
    currentPhase = problem_data.phaseSeq(2);
    problem_data.phaseSeq = bounding.get_gaitSeq(currentPhase,problem_data.n_Phases+1);
    problem_data.t_horizons     = bounding.get_timeSeq(currentPhase, problem_data.n_Phases);
    problem_data.N_horizons     = floor(problem_data.t_horizons./problem_data.dt);
    controller.updateHSDDP(problem_data);
end

%% save data
filename = 'simData';
save(filename, 'simTrajectory', 'totalCost');

%% Motor Analysis
% energy = motorAnalysis(simTrajectory);

%% Visualize motion
% construct graphics and visualize data
graphicsOptions.body_active = 1;
graphicsOptions.leg_active = 1;
graphicsOptions.push_active = 0;
graphicsOptions.GRF_acitive = 0;
graphicsOptions.showPlan  = 0;
graphicsOptions.view = '2D';
if options.gap_active
    graphicsOptions.gapActive = 1;
    graphicsOptions.gapLoc = 1.5;
    graphicsOptions.gapWidth = 0.4;
    graphicsOptions.filename = ['GapJumping', num2str(problem_data.n_WBPhases), num2str(problem_data.n_FBPhases)];
else
    graphicsOptions.gapActive = 0;
    graphicsOptions.filename = ['Bounding', num2str(problem_data.n_WBPhases), num2str(problem_data.n_FBPhases)];
end

graphics = Graphics(get3DMCParams(), WBMC2D);
graphics.setTrajectory({simTrajectory(:).X},{simTrajectory(:).U},{simTrajectory(:).Y});
graphics.visualize(graphicsOptions);
% % 
% graphics.plot('pos');
% graphics.plot('torque');
graphics.plot('GRF');