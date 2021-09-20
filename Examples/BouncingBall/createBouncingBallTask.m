function [phases, hybridT] = createBouncingBallTask(problem)
dt = problem.dt;
len_horizons = problem.len_horizons;

Ball = BouncingBall(dt);
bouncingCost = BouncingCost(dt);

phases(1) = Phase;
phases(2) = Phase;

hybridT(1) = Trajectory(2,1,1,len_horizons(1),dt);
hybridT(2) = Trajectory(2,1,1,len_horizons(2),dt);

phases(1).set_dynamics({@Ball.dynamics, @Ball.dynamics_par});
phases(1).set_resetmap({@Ball.resetmap, @Ball.resetmap_par});
phases(1).set_running_cost({@bouncingCost.running_cost, @bouncingCost.running_cost_par});
phases(1).set_terminal_cost({@(x) bouncingCost.terminal_cost(x,1),... 
                             @(x) bouncingCost.terminal_cost_par(x,1)});
phases(1).add_terminal_constraints(BouncingConstraint(1));


phases(2).set_dynamics({@Ball.dynamics, @Ball.dynamics_par});
phases(2).set_resetmap({@Ball.resetmap, @Ball.resetmap_par});
phases(2).set_running_cost({@bouncingCost.running_cost, @bouncingCost.running_cost_par});
phases(2).set_terminal_cost({@(x) bouncingCost.terminal_cost(x,2),... 
                             @(x) bouncingCost.terminal_cost_par(x,2)});
end