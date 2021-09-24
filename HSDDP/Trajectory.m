classdef Trajectory < handle
    properties
        len
        Ubar
        Xbar
        U
        X
        Y                
        G
        H
        K
        dU
        V
        dV
        l
        lpar
        phi
        phi_par
        A
        B
        C
        D
        Px
        dt
    end
    
    methods 
        function T = Trajectory(xsize, usize, ysize, N_horizon, dt)
            T.len = N_horizon;
            T.Ubar   = repmat({zeros(usize, 1)}, 1, N_horizon - 1);
            T.Xbar   = repmat({zeros(xsize, 1)}, 1, N_horizon);
            T.U      = repmat({zeros(usize, 1)}, 1, N_horizon - 1);
            T.dU     = repmat({zeros(usize, 1)}, 1, N_horizon);
            T.X      = repmat({zeros(xsize, 1)}, 1, N_horizon);
            T.Y      = repmat({zeros(ysize, 1)}, 1, N_horizon);
            T.G      = repmat({zeros(xsize, 1)}, 1, N_horizon);
            T.H      = repmat({zeros(xsize, xsize)}, 1, N_horizon);
            T.K      = repmat({zeros(usize, xsize)}, 1, N_horizon);
            T.A      = repmat({zeros(xsize, xsize)}, 1, N_horizon);
            T.B      = repmat({zeros(xsize, usize)}, 1, N_horizon);
            T.C      = repmat({zeros(ysize, xsize)}, 1, N_horizon);
            T.D      = repmat({zeros(ysize, usize)}, 1, N_horizon);
            T.Px     = [];
            T.l      = repmat({0}, 1, N_horizon - 1);
            T.lpar   = repmat({RCostPartialData(xsize, usize, ysize)}, 1, N_horizon - 1);
            T.phi    = 0;
            T.phi_par = TCostPartialData(xsize);
            T.V      = 0;
            T.dV     = 0;           
            T.dt     = dt;
        end    
        
        function updateNominal(T)
            T.Xbar = T.X;
            T.Ubar = T.U;
            T.dU     = repmat({zeros(size(T.dU{1}))}, 1, length(T.dU));
        end

        function append(T, Traj)
            % Researved for implementation
        end
    end
end