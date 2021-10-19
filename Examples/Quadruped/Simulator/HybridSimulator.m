classdef HybridSimulator < handle
    properties
        robot
        dt
        controller % function handle
    end

    methods
        function Sim = HybridSimulator(dt_in, option)
            Sim.dt = dt_in;            
            Sim.robot = Quadruped(dt_in,option.has_motor, option.DAE_reg_method);
            Sim.robot.gen_function_handles();
        end

        function set_controller(Sim, controller_in)
            Sim.controller = controller_in;
        end

        function [u_fine, x_fine] = run_lowlevel_control(Sim, x, ur, xr, ctact)
            % x-> current state
            % xr-> reference state
            x_fine = zeros(36, 34);
            u_fine = zeros(12, 33);
            x_fine(:,1) = x;
            for k = 1:33                
                u_fine(:,k) = Sim.controller(x_fine(:,k), xr);
                [x_fine(:,k+1), ~] = Sim.robot.dynamics_rpy(x_fine(:,k), u_fine(:,k), ctact);
            end
        end

        function simulate_new(Sim, hybridT, hybridR, ctact)
            % hybridT-> hybrid trajectory
            % hybridR-> hybrid trajectory reference
            % ctact -> 1x(n+1) cell array of contact status
            n_phase = length(hybridT);
            x0 = hybridT{1}.Xbar{1};            
            for i = 1:n_phase
                X = hybridT{i}.Xbar;
                X{1} = x0;
                U = hybridT{i}.Ubar;
                Y = hybridT{i}.Y;
                Xd = hybridR{i}.xd;
                Ud = hybridR{i}.ud;
                len_horizon = hybridR{i}.len;
                k = 1;
                for j = 1:len_horizon - 1
                    x = X{k};
                    xd = Xd{j};
                    ud = Ud{j};
                    [u_seg, x_seg] = Sim.run_lowlevel_control(x, ud, xd, ctact{i});
                    U(k:k+32) = num2cell(u_seg, 1);
                    X(k:k+33) = num2cell(x_seg, 1);
                    k = k + 33;
                end
                hybridT{i}.Xbar = X;
                hybridT{i}.Ubar = U;
                hybridT{i}.Y = Y;
                if i < n_phase
                    x0 = Sim.robot.resetmap(X{end}, ctact{i+1});
                end
            end
        end

        function simulate(Sim, hybridT, hybridR, ctact)
            % hybridT-> hybrid trajectory
            % hybridR-> hybrid trajectory reference 
            % ctact -> 1x(n+1) cell array of contact status
            n_phase = length(hybridT);
            x0 = hybridT{1}.Xbar{1};
            for i = 1:n_phase
                X = hybridT{i}.Xbar;
                X{1} = x0;
                U = hybridT{i}.Ubar;
                Y = hybridT{i}.Y;
                Xd = hybridR{i}.xd;
                Ud = hybridR{i}.ud;                
                len_horizon = hybridR{i}.len;
                for k = 1:len_horizon - 1
                    x = X{k};
                    xd = Xd{k};                    
                    u = Ud{k} + Sim.controller(x, xd);            
                    [xnext, y] = Sim.robot.dynamics_rpy(x, u, ctact{i});
                    X{k+1} = xnext;
                    U{k} = u;
                    Y{k} = y;
                end
                hybridT{i}.Xbar = X;
                hybridT{i}.Ubar = U;
                hybridT{i}.Y = Y;
                if i < n_phase
                    x0 = Sim.robot.resetmap(X{end}, ctact{i+1});
                end
            end
        end
    end
end