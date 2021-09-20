classdef PlanarQuadruped < BaseDyn
    properties (SetAccess=private, GetAccess=public)
        dt
        qsize = 7; 
        xsize = 14;
        usize = 4; 
        ysize = 4;          
        S = [zeros(3, 4);
             eye(4)];
        e = 0;  % elastic coefficient        
    end
    
    properties % link geometry and inertia parameters
        bodyMass
        bodyLength
        bodyHeight
        bodyWidth
        bodyCoM
        bodyRotInertia
        hipLinkLength
        hipLinkMass
        hipLinkCoM
        hipRotInertia
        hipLoc
        kneeLoc
        kneeLinkLength
        kneeLinkMass
        kneeLinkCoM
        kneeRotInertia   
        robotMass
    end
    
    properties % robot model (spatial v2)
        model
    end
            
    methods % constructor
        function Quad = PlanarQuadruped(dt)
            Quad.dt = dt;
        end
    end
    
    methods 
        function buildModel(Quad)
            % This function creates a full planar quadruped model structure
            % The 0-configuration is with legs straight down, cheetah
            % pointed along the +x axis of the ICS.
            % The ICS has +z up, +x right, and +y inner page
            % Planar model has 7 DoFs, x, z translation, rotation around y
            % and front (back) hip and knee rotations
            %% model initilization
            robot.NB = 7;                                  % number of moving bodies (2 fictitious bodies for trunk translations)
            robot.parent  = zeros(1,robot.NB);             % parent body indices
            robot.Xtree   = repmat({eye(6)},robot.NB,1);   % coordinate transforms
            robot.jtype   = repmat({'  '},robot.NB,1);     % joint types
            robot.I       = repmat({zeros(6)},robot.NB,1); % spatial inertias
            robot.gravity = [0 0 -9.81]';              % gravity acceleration vec
            
            nb = 0;                                        % current body index
            
            %% trunk translation x (body num 1) (massless)
            nb = nb + 1;
            robot.parent(nb) = nb - 1;
            robot.Xtree{nb} = eye(1);
            robot.jtype{nb} = 'Px';
            robot.I{nb} = mcI(0, zeros(3,1), zeros(3,3));
            
            %% trunk translation z direction (body num 2) (massless)
            nb = nb + 1;
            robot.parent(nb) = nb - 1;
            robot.Xtree{nb} = eye(1);
            robot.jtype{nb} = 'Pz';
            robot.I{nb} = mcI(0, zeros(3,1), zeros(3,3));
            
            %% trunck rotation about y direction (body num 3)
            nb = nb + 1;
            robot.parent(nb) = nb - 1;
            robot.Xtree{nb} = eye(1);
            robot.jtype{nb} = 'Ry';
            robot.I{nb} = mcI(Quad.bodyMass, Quad.bodyCoM, Quad.bodyRotInertia);
            
            nbase = nb; % floating base index for attaching two children links (hip links)
            NLEGS = 2;
            
            for i = 1:NLEGS
                %% Hip link
                nb = nb + 1;
                robot.parent(nb) = nbase; % parent of the hip link is base
                robot.Xtree{nb} = plux(ry(0), Quad.hipLoc{i});  % translation (half the body length)
                robot.jtype{nb} = 'Ry';
                robot.I{nb} = mcI(Quad.hipLinkMass, Quad.hipLinkCoM, Quad.hipRotInertia);
                
                %% Knee link
                nb = nb + 1;
                robot.parent(nb) = nb - 1; % parent of the knee link is hip link
                robot.Xtree{nb} = plux( ry(0), Quad.kneeLoc);    % translation (length of hip link)
                robot.jtype{nb} = 'Ry';
                robot.I{nb} = mcI(Quad.kneeLinkMass, Quad.kneeLinkCoM, Quad.kneeRotInertia);
            end
            Quad.model = robot;
        end                      
    end
    
    methods
        function [x_next, y] = dynamics(Quad,x,u,mode)
            q   = x(1:7, 1);
            qd  = x(8:14,1);
            
            
            % Get inertia matrix, and coriolis,centrifugal,gravity marix
            [H, C] = HandC(Quad.model, q, qd);                        
            
            % Get contact jacobians
            [J,Jd] = Quad.getFootJacobian(x, mode);
            
            % assemble KKT matrix
            if isempty(Jd)
                K = H;
                b = Quad.S*u - C;
            else
                K = [H, -J';J, zeros(size(J,1),size(J,1))];
                b = [Quad.S*u - C ; -Jd*qd];
            end
                                    
            % KKT inverse
            f_KKT = K\b;
            
            % continuous dynamics
            fcont = [qd; f_KKT(1:7,1)];
            
            % discrete dynamics
            x_next = x + fcont*Quad.dt;
            
            % output
            y = zeros(4,1);
            if length(f_KKT) > 7
                ycontact = f_KKT(8:end,1);
            end
            switch mode
                case 1 % BS
                    y = [zeros(2,1); ycontact];
                case {2,4} % FL1 and FL2
                    y = zeros(4,1);
                case 3 % FS
                    y = [ycontact; zeros(2,1)];                                
                case 5 % double stance
                    y = ycontact;
            end  
        end
        
        function x_next = resetmap(Quad,x,next_mode)
            qpre = x(1:7,1);        % pre-impact joint angle
            qdpre = x(8:end,1);     % pre-impact joint vel
            qpost = qpre;
            
            [H, ~] = HandC(Quad.model, qpre, qdpre);
            
            % Get contact Jacobians
            [J, ~] = Quad.getFootJacobian(x, next_mode);
            
            % Assemble impulse KKT matrix
            if isempty(J)
                K = H;
                b = H*qdpre;
            else
                K = [H, -J';J, zeros(size(J,1),size(J,1))];
                b = [H*qdpre;  -Quad.e*J*qdpre];
            end            
            
            % KKT inverse
            P_KKT = K\b;
            
            % Post state
            if any(next_mode == [2 4])
                qdpost = qdpre;
            else
                qdpost = P_KKT(1:7, 1);
            end                        
            x_next = [qpost; qdpost];
            
            % Output
            if length(P_KKT) > 7
                impulse = P_KKT(8:end,1);
            end
            y = zeros(4,1);
            switch next_mode
                case 1 % next mode is BS
                    y = [zeros(2,1); impulse];
                case 3 % next mode is FS
                    y = [impulse; zeros(2,1)];                    
                otherwise  
                    % For simplicity, y = 0 at smooth transition. This doesn't affect the
                    % optimization algorithm. An accurate way to do this is
                    % to use the smooth dynamics. Eq(11) in HSDDP paper.
                    y = zeros(4, 1);
            end  
        end
        function x_next = contractmap(Quad,x,next_mode)
            Pr = [eye(3), zeros(3,11);
                zeros(3,7),eye(3),zeros(3,4)];
            x_next = Quad.resetmap(x, next_mode);
            x_next = Pr*x_next;
        end
    end
            
    methods
        function [A,B,C,D] = dynamics_par(Quad,x,u,mode)
            q   = x(1:7, 1);
            qd  = x(8:end, 1);
            
            [H,  C] = HandC(Quad.model,q,qd);
            
            [J,  Jd] = Quad.getFootJacobian(x, mode);
            
            [Hx, Cx] = FreeDynamics_par(x);
                                    
            [Jx, Jdx] = Quad.getFootJacobianPar(x, mode);                                                         
            
            % KKT and KKT partials
            qdx = [zeros(7), eye(7)];
            if isempty(J)
                K  = H;
                b  = Quad.S*u - C;
                Kx = Hx;
                bx = -Cx;
                bu = Quad.S*eye(Quad.usize);
            else
                K = [H, -J';J, zeros(size(J,1),size(J,1))];
                b = [Quad.S*u - C; -Jd*qd];
                Kx = [Hx,  -permute(Jx,[2,1,3]);
                      Jx, zeros(size(Jx,1),size(Jx,1),Quad.xsize)];
                bx = [ -Cx;
                       -getMatVecProdPar(Jd,Jdx,qd,qdx)];
                bu = [ Quad.S*eye(Quad.usize)
                       zeros(size(Jd,1), Quad.usize)];
            end
            
            
           % FD partials
           [qddx, qddu, GRFx, GRFu] = getFDPar(K,Kx,b,bx,bu); % g represents GRF if any. For a swing leg gx gu would be empty  
           
           % state-space dynamics partials
           qdu = zeros(7, Quad.usize);
           fcon_x = [qdx; qddx]; fcon_u = [qdu; qddu];
           A = eye(Quad.xsize) + fcon_x*Quad.dt;
           B = fcon_u*Quad.dt;
           C = zeros(Quad.ysize, Quad.xsize);
           D = zeros(Quad.ysize, Quad.usize);           
           % adapt output partial according to current mode
           switch mode
               case 1 % BS
                  C(3:4, :) = GRFx; D(3:4, :) = GRFu;
               case 3 % FS
                  C(1:2, :) = GRFx; D(1:2, :) = GRFu;               
               case 5 % DS
                  C = GRFx; D = GRFu;               
           end
        end        
        function Px = resetmap_par(Quad,x,next_mode)
            qpre = x(1:7, 1);
            qdpre = x(8:end, 1);
            
            [H,  ~] = HandC(Quad.model, qpre, qdpre);
            
            [J,  ~] = Quad.getFootJacobian(x, next_mode);
            
            [Hx, ~] = FreeDynamics_par(x);
                                    
            [Jx, ~] = Quad.getFootJacobianPar(x, next_mode);
            
            % KKT and KKT partials
            qpost_x = [eye(7), zeros(7)];
            qdpre_x = [zeros(7), eye(7)];
            if ~isempty(J)
                K = [H, -J';J, zeros(size(J,1),size(J,1))];
                b = [H*qdpre; -Quad.e*J*qdpre];
                Kx = [Hx,  -permute(Jx,[2,1,3]);
                      Jx, zeros(size(Jx,1),size(Jx,1),Quad.xsize)];            
                bx = [getMatVecProdPar(H, Hx, qdpre, qdpre_x);
                      -getMatVecProdPar(Quad.e*J,Quad.e*Jx,qdpre,qdpre_x)];
            else
                K = H;
                b = H*qdpre;
                Kx = Hx;            
                bx = getMatVecProdPar(H, Hx, qdpre, qdpre_x);
            end
            
            if any(next_mode == [1 3])
                [qdpost_x, ~] = getFDPar(K,Kx,b,bx);
            else
                qdpost_x = qdpre_x;
            end            
            
            % State-space impact dyanmics partials
            Px = [qpost_x; qdpost_x];
        end
        function Px = contractmap_par(Quad,x,next_mode)
            Pr = [eye(3), zeros(3,11);
                zeros(3,7),eye(3),zeros(3,4)];
            Px = Quad.resetmap_par(x, next_mode);
            Px = Pr * Px;
        end
    end
      
    methods
        function T          = getKinematics(Quad,q)
            % This function computes the homogeneous transformation of each joint-fixed
            % frame w.r.t. Inertia coordinate system (IC)
            % Link index convention: body 1, f hip 2, f knee 3, b hip 4, b knee 5
            % q(1):x  q(2):y  q(3):roty
            % q(4):theta1 hip  q(5):theta2 knee (front leg)
            % q(6):theta1 hip  q(7):theta2 knee (back leg)                       
            
            %% kinematics
            % preallocate to save memory
            T  =  repmat({eye(4)}, [5, 1]);
            
            % body Pos and Orientation w.r.t IC
            T{1}    =  Trans([q(1), 0, q(2)])*RotY(q(3));
            
            % body frame to front hip frame
            T12     =  Trans(Quad.hipLoc{1})*RotY(q(4));
            
            % front hip frame to front knee frame
            T23     =  Trans(Quad.kneeLoc)*RotY(q(5));
            
            % body frame to back hip frame
            T14     =  Trans(Quad.hipLoc{2})*RotY(q(6));
            
            % back hip frame to back knee frame
            T45     =  Trans(Quad.kneeLoc)*RotY(q(7));
            
            % front hip frame w.r.t IC
            T{2}    =  T{1}*T12;
            
            % front knee frame w.r.t IC
            T{3}    =  T{2}*T23;
            
            % back hip frame w.r.t IC
            T{4}    = T{1}*T14;
            
            % back knee frame w.r.t IC
            T{5}    = T{4}*T45;
        end
        
        function ICSPos     = getPosition(Quad,q,linkidx, contactLoc)
            % This function gets the position of a contact point w.r.t. ICS.
            % ContactLoc is defined in local frame
            % Link index convention: body 1, f hip 2, f knee 3, b hip 4, b knee 5,                                   
            T  = Quad.getKinematics(q); ICSPos = [];
            for i = 1:size(contactLoc, 2)
                contactLoc_aug = [contactLoc(1,i), 0, contactLoc(2,i), 1]';
                ICSPos = [ICSPos,T{linkidx}*contactLoc_aug];
            end            
                        
            % remove y component and homogeneuous component
            ICSPos([2,4],:) = [];
        end
        
        function BodyPos  = getPositionBodyFrame(Quad,q,linkidx, contactLoc)
            
            T  = Quad.getKinematics(q); BodyPos = [];
            
            for i = 1:size(contactLoc,2)
                contactLoc_aug = [contactLoc(1,i), 0, contactLoc(2,i), 1]';
                BodyPos = [BodyPos, T{1}\T{linkidx}*contactLoc_aug];
            end
            
            % remove y component and homogeneuous component
            BodyPos([2,4],:) = [];
        end
        
        function ICSPos     = getFootPosition(Quad, q, mode)
             % Get contact jacobian and its partial according to mode spec
            ICSPos = []; 
            linkidx = []; contactLoc = [];            
            switch mode
                case 1
                    linkidx = 5; contactLoc = [0,-Quad.kneeLinkLength]';
                case 3
                    linkidx = 3; contactLoc = [0,-Quad.kneeLinkLength]'; % kneeLoc column vec                
                case {2,4}
                    linkidx = []; contactLoc = [];
                case 5
                    linkidx = [3, 5];
                    contactLoc = [0,-Quad.kneeLinkLength;
                                  0,-Quad.kneeLinkLength]';
            end
            
            for cidx = 1:length(linkidx)
                ICSPos = [ICSPos,Quad.getPosition(q, linkidx(cidx), contactLoc(:,cidx))];                
            end
        end
        
        function [J,Jd]     = getJacobian(Quad, x, linkidx, contactLoc)
            % This function computes Jacobian and Jacobian Derivative for
            % contact point on link linkidx.
            % contactLoc is in local frame
            % This function would be implemented with Spatial Vector method in future
            % release
            switch linkidx
                case 1
                    [J,Jd] = Link1Jacobian(x, contactLoc);
                case 2
                    [J,Jd] = Link2Jacobian(x, contactLoc);
                case 3
                    [J,Jd] = Link3Jacobian(x, contactLoc);
                case 4
                    [J,Jd] = Link4Jacobian(x, contactLoc);
                case 5
                    [J,Jd] = Link5Jacobian(x, contactLoc);
            end
        end       
        
        function [Jx,Jdx]   = getJacobianPar(Quad,x,linkidx,contactLoc)
            switch linkidx
                case 1
                    [Jx,Jdx] = Link1Jacobian_par(x, contactLoc);
                case 2
                    [Jx,Jdx] = Link2Jacobian_par(x, contactLoc);
                case 3
                    [Jx,Jdx] = Link3Jacobian_par(x, contactLoc);
                case 4
                    [Jx,Jdx] = Link4Jacobian_par(x, contactLoc);
                case 5
                    [Jx,Jdx] = Link5Jacobian_par(x, contactLoc);
            end
        end
        
        function [J,Jd]     = getFootJacobian(Quad, x, mode)
            % Get contact jacobian and its partial according to mode spec
            J = []; Jd = [];
            linkidx = []; contactLoc = [];            
            switch mode
                case 1
                    linkidx = 5; contactLoc = [0,-Quad.kneeLinkLength]';
                case 3
                    linkidx = 3; contactLoc = [0,-Quad.kneeLinkLength]'; % kneeLoc column vec                
                case {2,4}
                    linkidx = []; contactLoc = [];
                case 5
                    linkidx = [3, 5];
                    contactLoc = [0,-Quad.kneeLinkLength;
                                  0,-Quad.kneeLinkLength]';
            end
            
            for cidx = 1:length(linkidx)
                [J_cidx, Jd_cidx] = Quad.getJacobian(x, linkidx(cidx), contactLoc(:,cidx));
                J   = [J; J_cidx]; 
                Jd  = [Jd; Jd_cidx];
            end
        end
        
        function [Jx, Jdx]  = getFootJacobianPar(Quad, x, mode)
            Jx = []; Jdx = [];
            switch mode
                case 1
                    linkidx = 5; contactLoc = [0,-Quad.kneeLinkLength]';
                case 3
                    linkidx = 3; contactLoc = [0,-Quad.kneeLinkLength]'; % kneeLoc column vec                
                case {2,4}
                    linkidx = []; contactLoc = [];
                case 5
                    linkidx = [3, 5];
                    contactLoc = [0,-Quad.kneeLinkLength;
                                  0,-Quad.kneeLinkLength]';
            end
            
            for cidx = 1:length(linkidx)
                [Jx_cidx, Jdx_cidx] = Quad.getJacobianPar(x, linkidx(cidx), contactLoc(:,cidx));
                Jx  = [Jx; Jx_cidx]; 
                Jdx  = [Jdx; Jdx_cidx];
            end
        end
                
        function Initialize_model(Quad,varargin)
            % do nothing for WB model
        end
    end
       
end