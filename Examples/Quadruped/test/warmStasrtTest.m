clear 
clc
% Process RL rollout to formulate desired trajectory
[hybridR, ctactSeq, dT] = process_high_level_data();
n_phase = length(hybridR);

% Create data for actual trajectory
hybridT = cell(1, n_phase);
dt = 0.001;
for i = 1:n_phase
    duration = (hybridR{i}.len - 1) * dT;
    len_horizon_fine = int16 (duration/dt);
    hybridT{i} = Trajectory(36,12,12,len_horizon_fine,dt);
end
% Assume that the actual trajectory and desired trajectory agree initially
hybridT{1}.Xbar{1} = hybridR{1}.xd{1};

% Select number of phases to simulate
n_phase_sim = 2;
hybridT_sim = hybridT(1:n_phase_sim);
hybridR_sim = hybridR(1:n_phase_sim);
ctactSeq_sim = ctactSeq(1:n_phase_sim);

% Instantiate simulator
option.has_motor = true;
sim = HybridSimulator(dt, option);
sim.set_controller(@JointPD); % Set the controller for simulator to use
sim.simulate_new(hybridT_sim, hybridR_sim, ctactSeq_sim); % perform simulation

%% convert cell array to matrix
X = [];
U = [];
Xr = [];
Ur = [];
for T = [hybridT_sim{:}]
    X = [X, cell2mat(T.Xbar)];
    U = [U, cell2mat(T.Ubar)];
end
for R = [hybridR_sim{:}]
    Xr = [Xr, cell2mat(R.xd)];
    Ur = [Ur, cell2mat(R.ud)];
end
% len_sim = 28;
% X = X(:,1:len_sim);
% Xr = Xr(:,1:len_sim);
% U = U(:,1:len_sim);
% Ur = Ur(:,1:len_sim);

%% visulize MC
visualizeMCTrajectory(full(X(:,1:2:end)));

%% plot state and control trajecories


qa = X(7:18,:);             % joint anlge trajectory 
qad = X(25:end,:);          % joint velocity trajectory
qa_r = Xr(7:18,:);          % joint anlge reference trajectory
qad_r = Xr(25:end, :);      % joint velocity reference trajectory
plot_joint_trajectories(qa, qa_r, dT, dt);
plot_jointvel_trajectories(qad, qad_r, dT, dt);
plot_torque_trajectories(U, Ur, dT, dt);
