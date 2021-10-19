clear 
clc
dt = 0.001;
len_horizon = 1000;
traj = {Trajectory(36,12,12,len_horizon,0.001)};
traj_ref = {TrajReference(36,12,12,len_horizon)};

qa_ref = [0, -0.65,1.569, 0, -0.65,1.569,0, -0.6, 1.8,0, -0.6, 1.8]';
% traj{1}.Xbar{1}(7:18) = qa_ref;
traj_ref{1}.xd = repmat({[zeros(6,1);qa_ref;zeros(18,1)]},1,len_horizon);

option.has_motor = 1;
option.DAE_reg_method = "projection"; % method to stabilize the DAE of non-slip constraints
sim = HybridSimulator(dt, option);
sim.set_controller(@JointPD);
ctact_status = [1,1,1,1];
sim.simulate(traj, traj_ref, {ctact_status});

X = cell2mat(traj{1}.Xbar);
U = cell2mat(traj{1}.Ubar);
Xr = cell2mat(traj_ref{1}.xd);
Ur = cell2mat(traj_ref{1}.ud);
visualizeMCTrajectory(X(:,1:30:end));

qa = X(7:18,:);             % joint anlge trajectory 
qad = X(25:end,:);          % joint velocity trajectory
qa_r = Xr(7:18,:);          % joint anlge reference trajectory
qad_r = Xr(25:end, :);      % joint velocity reference trajectory
% plot_joint_trajectories(qa, qa_r, dt, dt);
% plot_jointvel_trajectories(qad, qad_r,dt, dt);
% plot_torque_trajectories(U, Ur, dt, dt);
% 
% figure
% plot(dt*(0:length(qad)-1), X(3,:))