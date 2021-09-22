function [phases, hybridTraj, hybridRef] = createBoundingTask(problem)
phaseSeq = problem.phaseSeq;
len_horizons = zeros(length(phaseSeq));
for i = 1:length(len_horizons)
    len_horizons(i) = floor(phaseSeq(i).duration/problem.dt);
end

% Build dynamics object
wbModel = PlanarQuadruped(problem.dt);
build2DminiCheetah(wbModel);
fbModel = PlanarFloatingBase(problem.dt);
footholdPlanner = FootholdPlanner(fbModel);

% Build cost object
wbCost = WBCost(problem.dt);
fbCost = FBCost(problem.dt);


% Build multiple phases and trajectory data
for i = 1:problem.nPhase
    if i <= problem.nWBPhase
        hybridRef(i) = TrajReference(14,4,4,len_horizons(i));
        hybridTraj(i) = Trajectory(14,4,4,len_horizons(i),problem.dt);
    else
        hybridRef(i) = TrajReference(6,4,4,len_horizons(i));
        hybridTraj(i) = Trajectory(6,4,4,len_horizons(i),problem.dt);
    end
end

for i = 1:problem.nPhase
    phases(i) = Phase;
    mode = phaseSeq(i).mode;    
    ctact = phaseSeq(i).ctact;
    i_next = i + 1;
    if i_next > problem.nPhase
        i_next = 1;
    end
    nmode = phaseSeq(i_next).mode;
    nctact = phaseSeq(i_next).ctact;
    duration = phaseSeq(i).duration;
    if i <= problem.nWBPhase
        % Whole-body phase
        phases(i).set_dynamics({@(x,u) wbModel.dynamics(x,u,mode),...
                                @(x,u) wbModel.dynamics_par(x,u,mode)});
        phases(i).set_resetmap({@(x) wbModel.resetmap(x,nmode),...
                                @(x) wbModel.resetmap_par(x,nmode)}); 
        if i+1 > problem.nWBPhase
            phases(i).set_resetmap({@(x) wbModel.contractmap(x,nmode),...
                                    @(x) wbModel.contractmap_par(x,nmode)});
        end
        phases(i).set_running_cost({@(k,x,u,y) wbCost.running_cost(k,x,u,y,mode,hybridRef(i)),...
                                    @(k,x,u,y) wbCost.running_cost_partial(k,x,u,y,mode,hybridRef(i))});                                        
        phases(i).set_terminal_cost({@(x) wbCost.terminal_cost(x,mode,hybridRef(i)),...
                                     @(x) wbCost.terminal_cost_partial(x,mode,hybridRef(i))});
        phases(i).add_path_constraints(torque_limit());
        if any(mode == [1 3])
            phases(i).add_path_constraints(GRF_constraint(ctact));
        end
        if any(nmode == [1 3])
            phases(i).add_terminal_constraints(SwitchingConstraint(nctact));
        end
    else
        % Floating-base phase
        phases(i).set_dynamics({@(x,u) fbModel.dynamics(x,u,ctact),...
                                @(x,u) fbModel.dynamics_par(x,u,ctact)});
        phases(i).set_resetmap({@fbModel.resetmap,...
                                @fbModel.resetmap_par});
        phases(i).set_running_cost({@(k,x,u,y) fbCost.running_cost(k,x,u,y,mode,hybridRef(i)),...
                                    @(k,x,u,y) fbCost.running_cost_partial(k,x,u,y,mode, hybridRef(i))});
        phases(i).set_terminal_cost({@(x) fbCost.terminal_cost(x,mode,hybridRef(i)),...
                                     @(x) fbCost.terminal_cost_partial(x,mode,hybridRef(i))});
        if any(mode == [1 3])
            phases(i).set_additional_initialization(@(x) footholdPlanner.foothold_prediction(x,duration,problem.vd));
        end
    end   
end
end