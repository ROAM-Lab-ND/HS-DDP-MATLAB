function Jv = compute_foot_jacobian(robot, q, kneeLinkLength, leg)
% robot -> robot tree built with spatial_v2
% q -> generalized joint angle nx1 vector
% kneeLinkLength -> knee link length
% leg -> leg ID [0,1,2,3]->[FR, FL , HR, HL]
% Jv -> Foot Jacobian for linear velicity in worldframe

Xtree_knee_to_foot = plux(eye(3), [0, 0, -kneeLinkLength]');
% Get the jacobian expressed in foot frame
J_local = BodyJacobian(robot, q, LINKID.knee(leg), Xtree_knee_to_foot);

[~, ~, info] = HandC(robot, q, zeros(size(q)));
Xup = info.Xup;
X = eye(6);

j = LINKID.knee(leg);

while j > 0
    X = X * Xup{j};
    j = robot.parent(j);
end


[R, ~] = plux(X);

J_world = blkdiag(R', R') * J_local;
Jv = J_world(4:end, :);
end