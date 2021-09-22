function footJacobDer = JacobianSupport()
import casadi.*

params = getMiniCheetahParams();
robot = buildModel(params);

pos = SX.sym('p', [3, 1]);
rpy = SX.sym('r', [3, 1]);
qa = SX.sym('qa', [12, 1]);
vel = SX.sym('v', [3, 1]);
rpyrate = SX.sym('rr', [3, 1]);
qa_d = SX.sym('qad', [12, 1]);

q = [pos; rpy; qa];
qd = [vel; rpyrate; qa_d];



%% Generate Jacobian Derative funtion objects
footIdx = params.footIdx;
kneeLinkLength = params.kneeLinkLength;
J = cell(1,4);
Jd = cell(1,4);
for foot = 1:4
    J{foot} = compute_Jacobian(robot, q, footIdx(foot), [0, 0, -kneeLinkLength]');
    Jd{foot} = reshape(jacobian(reshape(J{foot}, [numel(J{foot}),1]),q)*qd, 3, robot.NB);
end
footJacobDer = Function('footJacobDerivative', {q, qd}, Jd);
end