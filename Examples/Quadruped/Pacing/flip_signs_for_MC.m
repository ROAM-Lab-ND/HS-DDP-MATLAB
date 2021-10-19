function traj_MC = flip_signs_for_MC(traj_A1)
% This function adpats A1 trajectory for Mini Cheetah
% All joint rotations for A1 exactly use right-hand rule to determine the
% direction of rotation, whereas hip and knee joints of Mini Cheetah have
% positive rotation around -y axis

% traj_A1 -> trajectory defined using A1 convention
% traj_MC -> trajectory defined using MC convention
X_A1 = cell2mat(traj_A1.xd);
pos_MC = X_A1(1:3, :);
rpy_MC = X_A1(4:6, :);
vel_MC = X_A1(19:21, :);
rpyrate_MC = X_A1(22:24, :);
qa_MC = X_A1(7:18, :);
qad_MC = X_A1(25:end, :);
% Flip the sign for hip rotation
qa_MC([2,5,8,11],:) = -qa_MC([2,5,8,11],:);
qad_MC([2,5,8,11],:) = -qad_MC([2,5,8,11],:);
% Flip the sign for knee rotaiton
qa_MC([3,6,9,12],:) = -qa_MC([3,6,9,12],:);
qad_MC([3,6,9,12],:) = -qad_MC([3,6,9,12],:);

% Set abad angles to zero
qa_MC([1,4,7,10],:) = 0;
qad_MC([1,4,7,10],:) = 0;

u_MC = cell2mat(traj_A1.ud);
% Flip actuation direction for hip
u_MC([2,5,8,11],:) = -u_MC([2,5,8,11],:);
% Flip actuation direction for knee
u_MC([3,6,9,12],:) = -u_MC([3,6,9,12],:);

% Set abad actuation to zero
u_MC([1,4,7,10],:) = 0;
X_MC = [pos_MC; rpy_MC; qa_MC; vel_MC; rpyrate_MC; qad_MC];
traj_MC = traj_A1;
traj_MC.xd = mat2cell(X_MC, 36, ones(1,traj_A1.len));
traj_MC.ud = mat2cell(u_MC, 12, ones(1,traj_A1.len));
end