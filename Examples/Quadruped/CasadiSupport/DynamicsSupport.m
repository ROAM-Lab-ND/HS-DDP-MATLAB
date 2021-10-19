function funcObjs = DynamicsSupport()
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
footIds = params.footIds;
kneeLinkLength = params.kneeLinkLength;
J = cell(1,4);
Jd = cell(1,4);
for foot = 1:4
    J{foot} = compute_Jacobian(robot, q, footIds(foot), [0, 0, -kneeLinkLength]');
    Jd{foot} = reshape(jacobian(reshape(J{foot}, [numel(J{foot}),1]),q)*qd, 3, robot.NB);
end

%% Generate function objects of dynamics partials
S = [zeros(6,12); eye(12)]; % control selection
funcObjs = cell(1, 15);
for i = 1:15
    ctact = arrayfun(@str2num, dec2bin(i, 4));
    cfoot_idx = find(ctact == 1); % index of foot in contact
    
    [H, C] = HandC(robot, q, qd);
    
    % Construct KKT contact dynamics
    Jc = [];
    Jcd = [];
    for foot = cfoot_idx
        Jc = [Jc; J{foot}];
        Jcd = [Jcd; Jd{foot}];
    end
    
    if isempty(Jc)
        K = H;
        b = S*u - C;
    else
        alpha = 8;
        K = [H, -Jc';Jc, zeros(size(Jc,1),size(Jc,1))];
        b = [S*u - C; -Jcd*qd - alpha*Jc*qd];
    end
    f_KKT = K\b;
    qdd = f_KKT(1:robot.NB, 1);
    lambda = SX.zeros(12, 1);
    for c = 1:length(cfoot_idx)
        fid = cfoot_idx(c);
        lambda(3*(fid-1)+1:3*fid) = f_KKT(robot.NB+ (3*(c-1)+1:3*c));
    end
    
    f = [qd; qdd];
    Ac = jacobian(f, [q; qd]);
    Bc = jacobian(f, u);
    Cc = jacobian(lambda, [q; qd]);
    Dc = jacobian(lambda, u);
    funcName = sprintf('dynPartial%d', i);
    funcObjs{i} = Function(funcName, {q, qd, u}, {Ac, Bc, Cc, Dc});
end
end