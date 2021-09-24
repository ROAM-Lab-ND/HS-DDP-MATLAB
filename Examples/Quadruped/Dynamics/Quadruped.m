classdef Quadruped < AbstractRobot
    properties (SetAccess=private, GetAccess=public)
        dt      
        e = 0  % elastic coefficient  
        model
        params
        footIdx
        qs = 18
        xs = 36
        us = 12
        ys = 12
    end
    
    properties
        dynamics_par_handle
        jacob_dot_handle
        resetmap_par_handle
    end
                  
    methods % constructor
        function robot = Quadruped(dt)
            robot.dt = dt;
            robot.params = getMiniCheetahParams();
            robot.model = buildModel(robot.params);
            robot.footIds = robot.params.footIds;  % index of foot in the kinematic tree  
            robot.dynamics_par_handle = DynamicsSupport();
            robot.footJacob_dot_handle = JacobianSupport();
            robot.resetmap_par_handle = ResetmapSupport();
        end
    end    
        
    methods
        function [qdd, y] = dynamics_rpy_cont(robot, x, u, ctact_status)
            % compute continuous-time dynanmics which use rpy for
            % orientation
            % input x: x = [q; qd] where q is generalized coordinate
            %       ctact: contact status 1x4 vector, [FR, FL, RR, RL]
            % return qdd: second derivative of generalized joints
            %        y: ground reaction force of all feet (zero for swing leg)
            
            q = x(1:robot.qs, 1);
            qd = x(robot.qs+1:end, 1);
            ctact_foot = find(ctact_status == 1); % index of foot in contact
            y = zeros(robot.ys, 1);
            
            % Collect Jacobians and Jacobian derivatives of all feet
            for foot = 1:4
                J{foot} = compute_Jacobian(robot.model, q, robot.footIds(foot), [0,0,-robot.params.kneeLinkLength]);
                Jd{foot} = robot.footJacob_dot_handle{foot}(q, qd);
            end
            
            [H, C] = HandC(robot.model, q, qd);
            
            % Construct KKT contact dynamics
            Jc = [J{ctact_foot}];
            Jcd = [Jd{ctact_foot}];
            S = [zeros(6,12); eye(12)]; % control selection
            if isempty(Jc)
                K = H;
                b = S*u - C;
            else
                K = [H, -Jc';Jc, zeros(size(Jc,1),size(Jc,1))];
                b = [S*u - C; -Jcd*qd];
            end
            f_KKT = K\b;
            qdd = f_KKT(1:robot.model.NB);
            lambda = f_KKT(robot.mode.NB+1:end);            
            for foot = ctact_foot
                y(3*(foot-1)+1:3*foot) = lambda(1:3);
                lambda(1:3) = [];
            end
        end
        
        function [qdd, y] = dynamics_SE3_cont(robot, ws, u, ctact)
            % Reserved for future implementation of quaternion dynamics
        end        
        
        function [x_next, y] = dynamics_rpy(robot, x, u, ctact)
            % Compute discrete-time dynamics where rpy used for orientation   
            qd = x(robot.qs+1:end);
            [qdd, y] = robot.dynamics_rpy_cont(x, u, ctact);         
            x_next =  x + [qd * robot.dt;
                           qdd * robot.dt];
           
        end
        
        function [ws_next, lambda] = dynamics_SE3(robot, ws, u, ctact)
            % Reserved for future implementation of quaternion dynamics
        end
        
        function [x_next, y] = resetmap(robot, x, impt_foot)
            % Compute the reset map given the contacts at the current and
            % next time steps
            % Input ws: whole-body state            
            %       ctact: contact status 1x4 vector at curent time step, [FR, FL, RR, RL]
            %       ctact_n: contact status at next time step            
            q = x(1:robot.qs, 1);
            qd = x(robot.qs+1:end, 1);
            y = zeros(12, 1);
            
            [q_next, qd_next, lambda] = quadruped_impact_dynamics(robot.model, robot.params, q, qd, impt_foot);            
            x_next = [q_next; qd_next];
            for foot = impt_foot
                y(3*(foot-1):3*foot, 1) = lambda(1:3);
                lambda(1:3) = [];
            end
        end                
    end    
    methods
        function [Ac,Bc,Cc,Dc] = dynamics_par_rpy_cont(robot, x, u, ctact_status)
            q = x(1:robot.qs);
            qd = x(robot.qs+1:end);           
            mode = bin2dec(num2str(ctact_status));
            [Ac, Bc, Cc, Dc] = robot.dynamics_par_handle{mode}(q, qd, u);
        end
        
        function [A, B, C, D] = dynamics_par_rpy(robot, ws, u, ctact)
            [Ac, Bc, Cc, Dc] = robot.dynamics_par_rpy_cont(ws, u, ctact);
            A = eye(size(Ac)) + Ac * robot.dt;
            B = Bc * robot.dt;
            C = Cc;
            D = Dc;
        end
        
        function Px = resetmap_par(robot, x, impt_foot)
            q = x(1:robot.qs, 1);
            qd = x(robot.qs+1:end, 1);     
            
            mode = bin2dec(num2str(impt_foot));
            Px = robot.resetmap_par_handle{mode}(q, qd);
        end
    end                
      
    methods                              
        function Initialize_model(robot,varargin)
            % do nothing for WB model
        end
    end       
end