classdef WBCost < handle
    properties
       dt
       Q
       R
       S
       Qf
    end
    
    methods % constructor
        function C = WBCost(dt)
            C.dt = dt;
            % weightings for WB (whole-body) model
            % BS
            C.Q{1}  = 0.01*diag([0,10,5,4,4,4,4,  2,1,.01,6,6,6,6]);
            C.R{1}  = .5*diag([5,5,1,1]);
            C.S{1}  = blkdiag(zeros(2), 0.3*eye(2));
            C.Qf{1}  = 100*diag([0,20,8,3,3,3,3,  3,2,0.01,5,5,0.01,0.01]);
            
            % FL1
            C.Q{2}  = 0.01*diag([0,10,5,4,4,4,4,  2,1,.01,6,6,6,6]);
            C.R{2}  = .5*eye(4);
            C.S{2}  = zeros(4);
            C.Qf{2} = 100*diag([0,20,8,3,3,3,3,  3,2,0.01,5,5,5,5]);
            
            % FS
            C.Q{3}  = 0.01*diag([0,10,5,4,4,4,4,  2,1,.01,6,6,6,6]);
            C.R{3}  = .5*diag([1,1,5,5]);
            C.S{3}  = blkdiag(0.15*eye(2), zeros(2));
            C.Qf{3} = 100*diag([0,20,8,3,3,3,3,  3,2,0.01,0.01,0.01,5,5]);
            
            % FL2
            C.Q{4}  = 0.01*diag([0,10,5,4,4,4,4,  2,1,.01,6,6,6,6]);
            C.R{4}  = .5*eye(4);
            C.S{4}  = zeros(4);
            C.Qf{4} = 100*diag([0,20,8,3,3,3,3,  3,2,0.01,5,5,5,5]);
        end
       
    end
    
    methods 
        function l = running_cost(C,k,x,u,y,mode,r)
            l = wsquare(x - r.xd{k}, C.Q{mode});
            l = l + wsquare(u - r.ud{k}, C.R{mode});
            l = l + wsquare(y - r.yd{k}, C.S{mode});
            l = l * C.dt;
        end
        function lpar = running_cost_partial(C, k, x, u, y, mode, r)
            lpar.lx    = 2 * C.dt * C.Q{mode} * (x-r.xd{k}); % column vec
            lpar.lu    = 2 * C.dt * C.R{mode} * (u-r.ud{k});
            lpar.ly    = 2 * C.dt * C.S{mode} * (y-r.yd{k});
            lpar.lxx   = 2 * C.dt * C.Q{mode};
            lpar.lux   = zeros(length(u),length(x));
            lpar.luu   = 2 * C.dt * C.R{mode};
            lpar.lyy   = 2 * C.dt * C.S{mode};
        end
    end
    
    methods       
        function phi = terminal_cost(C, x, mode, r)
            phi = 0.5* wsquare(x - r.xd{end}, C.Qf{mode});
        end
        function phi_par = terminal_cost_partial(C, x, mode, r)
            phi_par.G = C.Qf{mode} * (x - r.xd{end});
            phi_par.H = C.Qf{mode};
        end                                    
    end    
end