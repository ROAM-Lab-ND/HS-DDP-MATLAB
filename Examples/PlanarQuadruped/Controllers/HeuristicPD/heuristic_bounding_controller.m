function heuristic_bounding_controller(hybridT, problem)
nWBPhase = problem.nWBPhase;
phaseSeq = problem.phaseSeq;
if nWBPhase <= 0
    fprintf('No whole-body trajectory is found. May need to check problem setup \n');
    fprintf('Heuristic controller is not executed \n');
    return
end
% Build quadruped model
model = PlanarQuadruped(problem.dt);
build2DminiCheetah(model);
% Get a library of modes for bounding
boundLib = BoundingModeLib();
%% flight phase reference angles
the_ref = [pi/4,-pi*7/12,pi/4,-pi*7/12]';
NomSpringLen = 0.2462;
kp = 5*diag([8,1,12,10]);
kd = 1*diag([1,1,1,1]);
Kspring = 2200;         % spring stiffness


for idx = 1:nWBPhase
    mode = phaseSeq(idx).mode;   
    nmode = boundLib.getNextMode(phaseSeq(idx)).mode;
    if mode == 1 %bs
        scale = 3;
        legid = 2;
    elseif mode == 3 %fs
        scale = 2.2;
        legid = 1;
    end
    for k = 1:hybridT(idx).len - 1
        xk  =   hybridT(idx).Xbar{k};
        qk  =   xk(1:7,1);
        switch mode
            case {1,3} 
                [Jc,~] = model.getFootJacobian(xk, mode);
                Jc(:,1:3) = [];
                v = getSpringVec(qk,model,legid);
                Fk = -v/norm(v)*Kspring*(norm(v)-NomSpringLen);
                uk = Jc'*Fk*scale;

            case {2,4}
                the_k = xk(4:7,1);
                thed_k = xk(11:end,1);
                uk = kp*(the_ref-the_k) - kd*thed_k; % first flight
        end
        [xk_next, ~] = model.dynamics(xk, uk, mode);
        hybridT(idx).Xbar{k+1} = xk_next;
        hybridT(idx).Ubar{k}   = uk;
    end
    if idx < nWBPhase
        xk_next = model.resetmap(hybridT(idx).Xbar{end}, nmode);
        hybridT(idx+1).Xbar{1} = xk_next;
    end
end
end

