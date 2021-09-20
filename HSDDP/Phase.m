classdef Phase< handle 
    properties
        dynamics
        resetmap
        dynamics_partial
        resetmap_partial
        running_cost
        terminal_cost
        running_cost_partial
        terminal_cost_partial        
        constraint
        additional_initialization = [];
        AL_params = [];
        ReB_params = [];
    end      
    
    methods 
        % Default constructor
        function P = Phase()
            P.constraint = ConstraintContainer();
        end
    end
    
    methods
        function set_dynamics(P, dynamics_funcs)
            % dynamics_funcs: 1x2 cell array of function handles of dynamics and dynamics
            % partials 
            P.dynamics = dynamics_funcs{1};
            P.dynamics_partial = dynamics_funcs{2};
        end
        function set_resetmap(P, reset_funcs)
            P.resetmap = reset_funcs{1};
            P.resetmap_partial = reset_funcs{2};
        end
        function set_running_cost(P, rcost_funcs)
            P.running_cost = rcost_funcs{1};
            P.running_cost_partial = rcost_funcs{2};
        end
        function set_terminal_cost(P, tcost_funcs)
            P.terminal_cost = tcost_funcs{1};
            P.terminal_cost_partial = tcost_funcs{2};
        end
        function add_path_constraints(P, pconstrObj)
            P.constraint.add_pathConstraint(pconstrObj);
        end
        function add_terminal_constraints(P, tconstrObj)
            P.constraint.add_terminalConstraint(tconstrObj);
        end
        function initialize_parameters(P)
            P.AL_params = P.constraint.get_al_params();
            P.ReB_params = P.constraint.get_reb_params();
        end       
        function set_additional_initialization(P, init_func)
            P.additional_initialization = init_func;
        end
    end
    methods (Access = private)
        function p_array = compute_pConstraints(P, x, u, y)           
            p_array = P.constraint.compute_pConstraints(x,u,y);
        end
        function t_array = compute_tConstraints(P, x)
            t_array = P.constraint.compute_tConstraints(x);
        end
    end
    methods
        function [h, success] = forwardsweep(P, traj, eps, options)
            N = traj.len;
            Xbar = traj.Xbar;
            Ubar = traj.Ubar;
            dU   = traj.dU;
            K = traj.K;
            traj.V = 0;
            success = 1;
            for k = 1:N-1
                x = traj.X{k};
                u = Ubar{k} + eps*dU{k} + K{k} * (x - Xbar{k});
                [x_next, y] = P.dynamics(x, u);
                l = P.running_cost(k, x, u, y);
                lpar = P.running_cost_partial(k, x, u, y);
                [A, B, C, D] = P.dynamics_partial(x, u);                
                if options.pconstraint_active
                    pconstrData = P.compute_pConstraints(x, u, y);
                    [l, lpar] = update_rcost_with_pconstraint(l, lpar, pconstrData, P.ReB_params, traj.dt);
                end   
                if any(isnan(u))
                    success = 0;
                    break;
                end
                traj.U{k} = u;
                traj.X{k+1} = x_next;
                traj.Y{k} = y;
                traj.l{k} = l;
                traj.lpar{k} = lpar;
                traj.A{k} = A;
                traj.B{k} = B;
                traj.C{k} = C;
                traj.D{k} = D;
                traj.V = traj.V + l;
            end
            phi = P.terminal_cost(traj.X{end});
            phi_par = P.terminal_cost_partial(traj.X{end});
            tconstrData = P.compute_tConstraints(traj.X{end});
            if options.tconstraint_active                
                [phi, phi_par] = update_tcost_with_tconstraint(phi, phi_par, tconstrData, P.AL_params);
            end
            traj.phi = phi;
            traj.phi_par = phi_par;
            traj.V = traj.V + phi;
            h = [];
            if ~isempty(tconstrData)
                h = [tconstrData(:).h]; % row vector if not empty
            end            
        end
        
        function success = backwardsweep(P, traj, Gprime, Hprime, dVprime, regularization)
            N = traj.len;
            success = 1;
            traj.G{end} = traj.phi_par.G + Gprime;
            traj.H{end} = traj.phi_par.H + Hprime;
            traj.dV = dVprime;
            for k = N-1:-1:1
                % Gradient and Hessian of value approx at next step
                Gk_next = traj.G{k+1};
                Hk_next = traj.H{k+1};
                
                % Dynamics linearization at current step
                Ak = traj.A{k}; Bk = traj.B{k};
                Ck = traj.C{k}; Dk = traj.D{k};
                
                % Compute Q info
                Qx  = traj.lpar{k}.lx  + Ak'*Gk_next         + Ck'*traj.lpar{k}.ly;
                Qu  = traj.lpar{k}.lu  + Bk'*Gk_next         + Dk'*traj.lpar{k}.ly;
                Qxx = traj.lpar{k}.lxx + Ck'*traj.lpar{k}.lyy*Ck  + Ak'*Hk_next*Ak;
                Quu = traj.lpar{k}.luu + Dk'*traj.lpar{k}.lyy*Dk  + Bk'*Hk_next*Bk;
                Qux = traj.lpar{k}.lux + Dk'*traj.lpar{k}.lyy*Ck  + Bk'*Hk_next*Ak;
                
                % regularization
                Qxx = Qxx + eye(size(Qxx))*regularization;
                Quu = Quu + eye(size(Quu))*regularization;
                
                [~, p] = chol(Quu-eye(size(Quu))*1e-9);
                if p ~= 0
                    success = 0;
                    break;
                end
                
                % Standard equations
                Quu_inv    = Sym(eye(size(Quu))/Quu);
                traj.dU{k} = -Quu_inv*Qu;
                traj.K{k}  = -Quu_inv*Qux;
                traj.G{k}  = Qx  - (Qux')*(Quu_inv* Qu );
                traj.H{k}  = Qxx - (Qux')*(Quu_inv* Qux);
                traj.dV    = traj.dV - Qu'*(Quu\Qu);
                
                if any(isnan(traj.dU{k})) || any(isnan(traj.G{k}))
                    error('error: du or Vx has nan at k = %d', k);
                end
                
            end
        end          
    end                       
end