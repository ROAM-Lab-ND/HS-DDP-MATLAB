classdef PlanarFloatingBase < BaseDyn
    properties (SetAccess=private, GetAccess=public)
        dt
        qsize = 3; 
        xsize = 6;        
        usize = 4; 
        ysize = 4;       
        p = zeros(4,1); % foothold location
    end
    
    properties % inertia params
        Inertia
        mass
    end
    
    methods
        % constructor
        function m = PlanarFloatingBase(dt) 
            m.dt = dt;  
            mc3D_param = get3DMCParams();
            mc2D_param = get2DMCParams(mc3D_param);
            m.recomputeInertia(mc2D_param);
        end
    end
    
    methods                
        function [x_next, y] = dynamics(m,x,u,s)  
            s = s(:);
            fcon    = FBDynamics(x,u,m.p,s);
            x_next  = x + fcon * m.dt;
            y       = u;
        end
        
        function [x_next, y] = resetmap(m,x)
            x_next = x;
            y = zeros(m.ysize, 1);
        end
    end
    
    methods
        function [A,B,C,D] = dynamics_par(m,x,u,s)   
            s = s(:);
            [fconx,fconu] = FBDynamics_par(x,u,m.p,s);
            A = eye(m.xsize) + fconx*m.dt;
            B = fconu*m.dt;
            C = zeros(m.ysize, m.xsize);
            D = eye(m.usize);
        end
        
        function  Px = resetmap_par(m,x)
            Px = eye(m.xsize);
            gx = zeros(m.ysize, m.xsize);
        end
    end
           
    methods
        function recomputeInertia(m, wbQuad)
            ihat = [1 0 0]';
            
            m.mass = wbQuad.robotMass;
            
            dist_CoM_hip = dot(wbQuad.hipLoc{1} + (ry(-pi/2))'*wbQuad.hipLinkCoM, ihat);
            dist_CoM_knee = dist_CoM_hip + norm(wbQuad.kneeLinkCoM);
            
            eqv_hipRotInertia =  wbQuad.hipRotInertia + wbQuad.hipLinkMass*dist_CoM_hip^2*0.85;
            eqv_kneeRotInertia = wbQuad.kneeRotInertia + wbQuad.kneeLinkMass*dist_CoM_knee^2*0.6;
            
            eqv_bodyRotInertia = wbQuad.bodyRotInertia + 2*eqv_hipRotInertia + 2*eqv_kneeRotInertia;           
            m.Inertia = eqv_bodyRotInertia(2,2);
        end
        function set_foothold(m, p)
            m.p = p;
        end
    end
end