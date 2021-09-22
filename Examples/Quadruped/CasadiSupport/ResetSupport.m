function funcObjs = ResetSupport()
import casadi.*
params = getMiniCheetahParams();
robot = buildModel(params);

pos = SX.sym('p', [3, 1]);
rpy = SX.sym('r', [3, 1]);
qa = SX.sym('qa', [12, 1]);
vel = SX.sym('v', [3, 1]);
rpyrate = SX.sym('rr', [3, 1]);
qa_d = SX.sym('qad', [12, 1]);
u = SX.sym('u', [12, 1]);

q = [pos; rpy; qa];
qd = [vel; rpyrate; qa_d];

%% Generate Jacobian Derative funtion objects
footIdx = params.footIdx;
kneeLinkLength = params.kneeLinkLength;
J = cell(1,4);
for foot = 1:4
    J{foot} = compute_Jacobian(robot, q, footIdx(foot), [0, 0, -kneeLinkLength]');
end

%%
funcObjs = cell(1, 15);
for i = 1:15
    imp_foot_status = arrayfun(@str2num, dec2bin(i, 4));    
    [qnext, qdnext, lambda] = quadruped_impact_dynamics(robot, params, q, qd, find(imp_foot_status == 1));
    R = [qnext; qdnext];
    Rx = jacobian(R, [q; qd]);
    lambdax = jacobian(lambda, [q; qd]);
    funcName = sprintf('resetPartial%d', i);
    funcObjs{i} = Function(funcName, {q, qd}, {Rx, lambdax});
end



end