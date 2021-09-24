function TouchdownSupport()
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
footIds = params.footIds;
kneeLinkLength = params.kneeLinkLength;

gen_params.mex = true;
gen_params.with_header = false;

pwdName = pwd;
dirName = '/home/wensinglab/HL/Code/HSDDP/MATLAB/Examples/Quadruped/Pacing/Constraint/mex/';
cd(dirName);
for foot = 1:4
    p_foot = forward_kinematics(robot, q, footIds(foot), [0,0,-kneeLinkLength]');
    h = p_foot(3);
    hx = jacobian(h, [q; qd]);
    funcName = sprintf('constr_par%d',foot);
    funcObj = Function(funcName, {q}, {hx});
    fileName = strcat(funcName, '.cpp');
    funcObj.generate(fileName, gen_params);    
    mex('-largeArrayDims', fileName);
end
cd(pwdName);
end