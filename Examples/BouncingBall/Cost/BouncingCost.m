classdef BouncingCost < handle
    properties
        dt
        R
        Q
        Qf
        S
    end
    methods
        function C = BouncingCost(dt)
            C.dt = dt;
            C.R = 0.5*dt;
            C.Q = zeros(2);
            C.S = 0;            
            C.Qf = 100 * eye(2);
        end
        function l = running_cost(C,k,x,u,y)
            l = C.R * u^2;
        end
        function lpar = running_cost_par(C,k,x,u,y)
            lpar = RCostPartialData(2,1,1);
            lpar.lu = 2*C.R*u;
            lpar.luu = 2*C.R;
        end
        function phi = terminal_cost(C,x,mode)
            if mode == 1
                phi = 0;
            else
                xd = [3, 0]';
                phi = wsquare(x-xd, C.Qf);
            end            
        end
        function phi_par = terminal_cost_par(C,x,mode)
            phi_par = TCostPartialData(2);
            xd = [3, 0]';
            if mode == 2
                phi_par.G = 2*C.Qf*(x - xd);
                phi_par.H = 2*C.Qf;
            end            
        end
    end
end