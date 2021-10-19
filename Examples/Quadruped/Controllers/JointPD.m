function u = JointPD(x, xd)
KP_leg = 2*diag([80, 80, 80]);
KD_leg = diag([2, 2, 2]);
% KP_leg = diag([0, 0, 0]);
% KD_leg = diag([0, 0, 0]);
KP = blkdiag(KP_leg, KP_leg, KP_leg, KP_leg);
KD = blkdiag(KD_leg, KD_leg, KD_leg, KD_leg);

qa = x(7:18,1);
qad = x(25:end, 1);
qa_r = xd(7:18,1);
qad_r = xd(25:end, 1);

u = -KP * (qa - qa_r) - KD * (qad - qad_r);
end