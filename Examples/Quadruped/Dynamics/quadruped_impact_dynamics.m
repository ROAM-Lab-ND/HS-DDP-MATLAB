function [q_next, qd_next, lambda] = quadruped_impact_dynamics(robot, params, q, qd, imp_foot)
% robot: quadurped model built with spatial_v2
% params: physical parameters of the quadruped
% q: 18x1 vector generalized coordinate before impact q = [pos; rpy; qa]
% qd: 18x1 vector generalized velocity before impact qd = [vel; rpyrate; qad];
% imp_foot_status: 1x4 vector of impact status (to touch down) of foot
% q_next: 18x1 vector generalized coordinate after impact
% qd_next: 18x1 vector generalized velocity after impact
% lambda: impulse of impact foot. Dimension varies depending on the number
% of impact foot

% Generate Jacobian Derative funtion objects
footIds = params.footIds;
kneeLinkLength = params.kneeLinkLength;
J = cell(1,4);
for foot = 1:4
    J{foot} = compute_Jacobian(robot, q, footIds(foot), [0, 0, -kneeLinkLength]');
end

% Compute mass matrix and Coriollis term assuming no
% constraints
[H, ~] = HandC(robot, q, qd);


% Construct KKT Impact dynamics
Jc = [];
for foot = 1:length(imp_foot)
    if imp_foot(foot)
        Jc = [Jc; J{foot}];
    end    
end
if isempty(Jc)
    K = H;
    b = H*qd;
else
    K = [H, -Jc';Jc, zeros(size(Jc,1),size(Jc,1))];
    b = [H*qd;  robot.e*Jc*qd];
end
% KKT inverse
R_KKT = K\b;
q_next = q;
qd_next = R_KKT(1:robot.NB);
lambda = R_KKT(robot.NB+1:end);
end