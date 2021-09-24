function [phases, hybridT, hybridRef] = createPacingTask()
[hybridRef, ctactSeq, dt] = process_high_level_data(); % ctactSeq has one more element than hybridRef

% formulate the trajectory optimization problem
nSeq = length(hybridRef);
len_horizons = zeros(1, nSeq);
for i = 1:nSeq
    len_horizons(i) = hybridRef{i}.len;
end

quad = Quadruped(dt);
cost = PacingCost(dt);
qs = quad.qs; 
xs = quad.xs; 
us = quad.us; 
ys = quad.ys;

for i = 1:nSeq
    hybridT(i) = Trajectory(xs, us, ys, len_horizons(i), dt);
    phases(i) = Phase();
    
    ctact_status = ctactSeq{i};
    ctact_status_next = ctactSeq{i+1};    
    impt_status = zeros(1, 4);
    for foot = 1:4
        if (~ctact_status(foot)) && ctact_status_next(foot)
            impt_status(foot) = 1;
        end
    end
    impt_foot = find(impt_status == 1);
    phases(i).set_dynamics({@(x, u) quad.dynamics_rpy(x, u, ctact_status),...
                            @(x, u) quad.dynamics_par_rpy(x, u, ctact_status)});    
    phases(i).set_running_cost({@(k,x,u,y) cost.running_cost(k,x,u,y,hybridRef{i}),...
                                @(k,x,u,y) cost.running_cost_partial(k,x,u,y,hybridRef{i})});
    phases(i).set_terminal_cost({@(x) cost.terminal_cost(x, hybridRef{i}),...
                                 @(x) cost.terminal_cost_partial(x, hybridRef{i})});
    if ~isempty(impt_foot)
        phases(i).set_resetmap({@(x) quad.resetmap(x, impt_foot),...
                                @(x) quad.resetmap_par(x, impt_foot)});
        phases(i).add_add_terminal_constraints(TouchdownConstraint(impt_foot));
    else
        phases(i).set_resetmap({@default_resetmap,...
                                @default_resetmap_partial});
    end
    
end

end