classdef Quadruped < AbstractRobot
    properties (SetAccess=private, GetAccess=public)
        dt      
        e = 0  % elastic coefficient  
        model
        params
        footIdx
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
            robot.footIdx = [9, 12, 15, 18];  % index of foot in the kinematic tree  
            robot.dynamics_par_handle = DynamicsSupport();
            robot.footJacob_dot_handle = JacobianSupport();
            robot.resetmap_par_handle = ResetmapSupport();
        end
    end    
    
    methods                                
        function Jd = compute_FootJacob_Der(robot, footid)
            % Compute derivative of foot Jacobian 
            % footid: Take one of values in robot.footIdx
            q = [ws.pos; ws.rpy; ws.q];
            qd = [ws.vel; ws.rpyrate; ws.qd];
            Jda = cell(1,4);
            % Collect all Jacobian derivatives
            [Jda{1}, Jda{2}, Jda{3}, Jda{4}] = footJacobDer(q, qd);
            Jd = Jda{robot.footIdx == footid};
        end
    end
    methods
        function [qdd, lambda] = dynamics_rpy_cont(robot, ws, u, ctact)
            % compute continuous-time dynanmics which use rpy for
            % orientation
            % input ws: whole-body state
            %       ctact: contact status 1x4 vector, [FR, FL, RR, RL]
            % return qdd: second derivative of generalized joints
            %       lambda: ground reaction force
            
            q = [ws.pos; ws.rpy;ws.q];
            qd = [ws.vel; ws.rpyrate; ws.qd];            
            cfoot_idx = find(ctact == 1); % index of foot in contact
            
            % Collect Jacobians and Jacobian derivatives of all feet
            for foot = 1:4
                Ja{foot} = compute_Jacobian(robot.model, q, robot.footIdx(foot), [0,0,-robot.params.kneeLinkLength]);
                Jda{foot} = robot.footJacob_dot_handle{foot}(q, qd);
            end
            
            [H, C] = HandC(robot.model, q, qd);
            
            % Construct KKT contact dynamics
            J = [Ja{cfoot_idx}];
            Jd = [Jda{cfoot_idx}];
            S = [zeros(6,12); eye(12)]; % control selection
            if isempty(J)
                K = H;
                b = S*u - C;
            else
                K = [H, -J';J, zeros(size(J,1),size(J,1))];
                b = [S*u - C; -Jd*qd];
            end
            f_KKT = K\b;
            qdd = f_KKT(1:robot.model.NB, 1);
            lambda = zeros(12, 1);
            if ~isempty(cfoot_idx)
                lambda(cfoot_idx(1):cfoot_idx(end)+2) = f_KKT(robot.model.NB+1:end);
            end
        end
        
        function [qdd, lambda] = dynamics_SE3_cont(robot, ws, u, ctact)
            % Reserved for future implementation of quaternion dynamics
        end        
        
        function [ws_next, lambda] = dynamics_rpy(robot, ws, u, ctact)
            % Compute discrete-time dynamics where rpy used for orientation
            qd = [ws.vel; ws.rpyrate; ws.qd];
            [qdd, lambda] = robot.dynamics_rpy_cont(ws, u, ctact);
            
            q_next = qd * robot.model.dt;
            qd_next = qdd * robot.model.dt;
            
            ws_next.pos = q_next(1:3);
            ws_next.rpy = q_next(4:6);
            ws_next.q = q_next(7:robot.model.NB);
            ws_next.vel = qd_next(1:3);
            ws_next.rpyrate = qd_next(4:6);
            ws_next.qd = qd_next(7:robot.model.NB);
        end
        
        function [ws_next, lambda] = dynamics_SE3(robot, ws, u, ctact)
            % Reserved for future implementation of quaternion dynamics
        end
        
        function [ws_next, y] = resetmap(robot, ws, ctact, ctact_n)
            % Compute the reset map given the contacts at the current and
            % next time steps
            % Input ws: whole-body state            
            %       ctact: contact status 1x4 vector at curent time step, [FR, FL, RR, RL]
            %       ctact_n: contact status at next time step            
            q = [ws.pos; ws.rpy;ws.q];
            qd = [ws.vel; ws.rpyrate; ws.qd];                                                
            % Find impact foot 
            imp_foot_idx = [];
            for foot = 1:4
                if (ctact(foot)==0) && (ctact_n(foot)==1) 
                % If current is swing and next is stance, we say this is
                % impact. This does not resolve the problem when impact
                % happens in between the time steps
                    imp_foot_idx(end+1) = foot;
                end
            end            
            [q_next, qd_next, lambda] = quadruped_impact_dynamics(robot.model, robot.params, q, qd, imp_foot_idx);
            ws_next = WBState;
            y = zeros(12, 1);
            ws_next.pos = q_next(1:3);
            ws_next.rpy = q_next(4:6);
            ws_next.q = q_next(7:robot.model.NB);
            ws_next.vel = qd_next(1:3);
            ws_next.rpyrate = qd_next(4:6);
            ws_next.qd = qd_next(7:robot.model.NB); 
            for fid = imp_foot_idx
                y(3*(fid-1):3*fid, 1) = lambda(1:3);
                lambda(1:3) = [];
            end
        end                
    end    
    methods
        function [Ac,Bc,Cc,Dc] = dynamics_par_rpy_cont(robot, ws, u, ctact)
            q = [ws.pos; ws.rpy;ws.q];
            qd = [ws.vel; ws.rpyrate; ws.qd];
            mode = bin2dec(num2str(ctact));
            [Ac, Bc, Cc, Dc] = robot.dynamics_par_handle{mode}(q, qd, u);
        end
        
        function [A, B, C, D] = dynamics_par_rpy(robot, ws, u, ctact)
            [Ac, Bc, Cc, Dc] = robot.dynamics_par_rpy_cont(ws, u, ctact);
            A = eye(size(Ac)) + Ac * robot.dt;
            B = Bc * robot.dt;
            C = Cc;
            D = Dc;
        end
        
        function Px = resetmap_par(robot, ws, ctact, ctact_n)
            q = [ws.pos; ws.rpy;ws.q];
            qd = [ws.vel; ws.rpyrate; ws.qd];                                                
            % Find impact foot 
            imp_foot_idx = [];
            for foot = 1:4
                if (ctact(foot)==0) && (ctact_n(foot)==1) 
                % If current is swing and next is stance, we say this is
                % impact. This does not resolve the problem when impact
                % happens in between the time steps
                    imp_foot_idx(end+1) = foot;
                end
            end       
            mode = bin2dec(num2str(imp_foot_idx));
            Px = robot.resetmap_par_handle{mode};
        end
    end                
      
    methods                              
        function Initialize_model(robot,varargin)
            % do nothing for WB model
        end
    end       
end