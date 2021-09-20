classdef HSDDP < handle
    properties %(Access = private)
        x0
        n_phases      
        len_phases
        phases
        hybridT     % array of phase trajectory        
    end
    
    properties
        V = 0;
        dV = 0;
        h_all      % terminal constraint collected in cell array
        maxViolation = 0;
    end
    
  
    methods %(constructor)
        function DDP = HSDDP(phases_in, hybridT_in)
            if nargin > 0
               DDP.Initialization(phases_in, hybridT_in);
            end
        end
        
        function Initialization(DDP, phases_in, hybridT_in)
            assert(length(phases_in) == length(hybridT_in), 'Fail to construct HSDDP. Dimensions of Phases and HybridTrajectories do not match.');
            DDP.phases  = phases_in;
            DDP.hybridT = hybridT_in;                        
            DDP.n_phases = length(phases_in);
            DDP.len_phases = [hybridT_in(:).len];            
            DDP.h_all       = cell(DDP.n_phases,1);
        end
    end
    
    methods
        function success = forwardsweep(DDP, eps, options)
            DDP.V = 0;
            success = 1;
            assert(isequal(DDP.x0, DDP.hybridT(1).Xbar{1}), 'Initial conditions do not match.');
            for idx = 1:DDP.n_phases                
                if idx == 1 
                    xinit = DDP.x0;                    
                else
                    xinit = DDP.phases(idx-1).resetmap(DDP.hybridT(idx-1).X{end});
                    DDP.hybridT(idx-1).Px = DDP.phases(idx-1).resetmap_partial(DDP.hybridT(idx-1).X{end});
                end
                DDP.hybridT(idx).X{1} = xinit;
                if ~isempty(DDP.phases(idx).additional_initialization)
                    % This executes foothold planner under the scene for trunk model
                    DDP.phases(idx).additional_initialization (xinit);                    
                end
                
                [DDP.h_all{idx}, success] = DDP.phases(idx).forwardsweep(DDP.hybridT(idx), eps, options);
                if ~success
                    break;
                end
                % Accumulates the total actual cost
                DDP.V = DDP.V + DDP.hybridT(idx).V;
            end
            % Maximum terminal constraint violation of all types over all
            % time instants
            DDP.maxViolation = max(cellfun(@(x) norm(x, Inf), DDP.h_all));
            if isempty(DDP.maxViolation)
                DDP.maxViolation = 0;
            end
        end
        
        function forwarditeration(DDP, options)
            eps = 1;
            Vprev = DDP.V;
            options.parCalc_active = 0;
            while eps > 1e-10                                                                 
                 success = DDP.forwardsweep(eps, options);
                 
                 if options.Debug                                        
                    fprintf('\t eps=%.3e \t cost change=%.3e \t min=%.3e\n',eps, DDP.V-Vprev, options.gamma* eps*(1-eps/2)*DDP.dV );
                    if ~success
                        fprintf('forward sweep fails because of Nan \n');
                        fprintf('reduce step size \n');
                    end
                 end
                 
                 if success
                     % If forward_sweep succeeds
                     % check if V is small enough to accept the step
                     if DDP.V < Vprev + options.gamma*eps*(1-eps/2)*DDP.dV
                         break;
                     end
                 end                 
                 
                 % Else backtrack
                 eps = options.alpha*eps;
            end
        end
        
        function success = backwardsweep(DDP, regularization)
            success = 1;
            for idx = DDP.n_phases:-1:1
                if idx == DDP.n_phases
                    Gprime = zeros(size(DDP.hybridT(idx).G{1}));
                    Hprime = zeros(size(DDP.hybridT(idx).H{1}));
                    dVprime = 0;
                else
                    Gprime = DDP.hybridT(idx+1).G{1};
                    Hprime = DDP.hybridT(idx+1).H{1};
                    dVprime = DDP.hybridT(idx+1).dV;
                    if ~isempty(DDP.phases(idx).resetmap)                        
                        [Gprime, Hprime] = impact_aware_step(DDP.hybridT(idx).Px, Gprime, Hprime);
                    end
                end                
                success = DDP.phases(idx).backwardsweep(DDP.hybridT(idx), Gprime, Hprime, dVprime, regularization);               
                if ~success
                    break;
                end
            end
            DDP.dV = DDP.hybridT(1).dV;
        end
                
                
        function reg = backwardsweep_regularization(DDP, reg, options)
            % backward sweep with regularization
            success = 0;
            while ~success
                if options.Debug
                    fprintf('\t reg=%.3e\n',reg);
                end
                if reg > 1e04
                    error('Regularization exeeds allowed value')
                end
                success = DDP.backwardsweep(reg);
                if success
                    break;
                end
                reg = max(reg*options.beta_reg, 1e-3);
            end
        end
        function [xopt, uopt, Kopt, Info] = solve(DDP, options)
            Info.ou_iters = 0;
            Info.in_iters = [];
            Info.max_violations = [];
            Info.al_params = [];
            
            xopt = cell(1,DDP.n_phases);
            uopt = cell(1,DDP.n_phases);
            Kopt = cell(1,DDP.n_phases);            
            DDP.initialize_params();           
            ou_iter = 0;     
            pconstraint_active = options.pconstraint_active;
            while 1 % Implement AL and ReB in outer loop
                ou_iter = ou_iter + 1;
                if  options.Debug
                    fprintf('====================================================\n');
                    fprintf('\t Outer loop Iteration %3d\n',ou_iter);
                end                
                % Initial sweep 
                if pconstraint_active
                    if ((DDP.maxViolation > 0.05) || (ou_iter == 1))
                        % Path constraint is not considered in the first AL
                        % iteration and when switching constraint violation is
                        % too large
                        options.pconstraint_active = 0;
                    else
                        options.pconstraint_active = 1;
                    end
                end
                
                Info.ou_iters = ou_iter;
                Info.al_params{end+1} = DDP.phases(:).AL_params;
                
                DDP.forwardsweep(1, options);
                DDP.updateNominalTrajectory();
                
                if ou_iter == 1
                    % Maximum violation with initial guess
                    Info.max_violations(end+1) = DDP.maxViolation;
                end
                
                Vprev = DDP.V;
                regularization = 0;       
                in_iter = 1;
                while 1 % DDP in inner loop
                    if  options.Debug
                        fprintf('================================================\n');
                        fprintf('\t Inner loop Iteration %3d\n',in_iter);
                    end                                        
                    
                    regularization = DDP.backwardsweep_regularization(regularization, options); 
                    % reduce regularization term
                    regularization = regularization/20;
                    if regularization < 1e-6
                        regularization = 0;
                    end
                    % linear search
                    DDP.forwarditeration(options);
                    % Update nominal trajectory once linear search is
                    % successful
                    DDP.updateNominalTrajectory();
                                        
                    if (in_iter>=options.max_DDP_iter) || (Vprev - DDP.V < options.DDP_thresh)
                        Info.in_iters(end+1) = in_iter;
                        Info.max_violations(end+1) = DDP.maxViolation;
                        break
                    end
                    in_iter = in_iter + 1;     
                    Vprev = DDP.V;                    
                end
                fprintf('Maximum terminal constraint violation %.4f\n',DDP.maxViolation);
                if (ou_iter>=options.max_AL_iter) || (DDP.maxViolation < options.AL_thresh)
                    if options.one_more_sweep
                        DDP.mod_AL_params_last_backsweep();
                        DDP.forwardsweep(1,options);
                        DDP.backwardsweep_regularization(regularization, options);
                    end
                    break;
                end
                DDP.update_ReB_params(options);                
                DDP.update_AL_params(options);
            end
                                   
            for idx = 1:DDP.n_phases
                xopt{idx} = DDP.hybridT(idx).Xbar;
                uopt{idx} = DDP.hybridT(idx).Ubar;
                Kopt{idx} = DDP.hybridT(idx).K;
            end
        end                 
    end
    
    methods                     
        function set_initial_condition(DDP, x0)
            DDP.x0 = x0;
        end
        
        function updateNominalTrajectory(DDP)
            for idx = 1:DDP.n_phases
                DDP.hybridT(idx).updateNominal();
            end
        end        
    end
    
    methods
        function initialize_params(DDP)
            for i = 1:DDP.n_phases
                DDP.phases(i).initialize_parameters();
            end
        end
        function update_ReB_params(DDP, options)
            if ~options.pconstraint_active
                % If path constraint is not active, do nothing
                return            
            end           
            for i = 1:DDP.n_phases
                % get ReB parameters for the current phase
                reb_params = DDP.phases(i).ReB_params;
                for j = 1:length(reb_params)
                    reb_params(j).eps = options.beta_ReB* reb_params(j).eps;
                    reb_params(j).delta = options.beta_relax*reb_params(j).delta;
                    if reb_params(j).delta < reb_params(j).delta_min
                        reb_params(j).delta = reb_params(j).delta_min;
                    end
                end
                DDP.phases(i).ReB_params = reb_params;
            end
        end
        function update_AL_params(DDP, options)
            if ~options.tconstraint_active
                % If terminal constraint is not active, do nothing
                return
            end
            for i = 1:DDP.n_phases
                h = DDP.h_all{i}; % get terminal constraints at current phase
                if (~isempty(DDP.phases(i).AL_params)) && (~isempty(h))
                    for j = 1:length(DDP.phases(i).AL_params)
                        lambda = DDP.phases(i).AL_params(j).lambda;
                        sigma = DDP.phases(i).AL_params(j).sigma;
                        lambda = lambda + h(j)*sigma;
                        if abs(h(j)) > 0.01
                            sigma = options.beta_penalty * sigma;
                        end
                        DDP.phases(i).AL_params(j).lambda = lambda;
                        DDP.phases(i).AL_params(j).sigma = sigma;
                    end
                end
            end
        end       
        function mod_AL_params_last_backsweep(DDP)
           % set all penalty to zero but keep the Lagrangian multiplier unchanged 
           for i = 1:DDP.n_phases
               for j = 1:length(DDP.phases(i).AL_params)
                   DDP.phases(i).AL_params(j).sigma = 0;
               end
           end
        end
    end           
end