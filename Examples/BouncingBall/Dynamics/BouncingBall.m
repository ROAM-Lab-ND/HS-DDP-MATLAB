classdef BouncingBall < handle
    properties 
        dt
        m
        g
        e
    end
    methods
        function B = BouncingBall(dt)
            B.dt = dt;
            B.m = 1;
            B.g = 9.81;
            B.e = 0.75;
        end
    end
    methods
        function [f, y] = dynamics_cont(B, x, u)
            % x 2x1 vector. x = [z, zdot]';
            % u is a scalar
            Ac = [0 1; 0 0];
            Bc = [0; 1/B.m];
            Cc = [1 0];
            Dc = 0;
            f = Ac * x + Bc * u - [0; B.g];
            y = Cc * x + Dc * u;
        end
        
        function [xnext, y] = dynamics(B,x,u)
            [f, y] = B.dynamics_cont(x,u);
            xnext = x + f * B.dt;
        end
        
        function [Ac,Bc,Cc,Dc] = dynamics_par_cont(B,x,u)
            Ac = [0 1; 0 0];
            Bc = [0; 1/B.m];
            Cc = [1 0];
            Dc = 0;
        end
        function [A,B,C,D] = dynamics_par(B,x,u)
            [Ac,Bc,Cc,Dc] = B.dynamics_par_cont(x, u);
            A = eye(size(Ac)) + Ac * B.dt;
            B = Bc * B.dt;
            C = Cc;
            D = Dc;
        end
        function xnext = resetmap(B, x)
            Px = [1 0; 0 -B.e];
            xnext = Px * x;
        end
        function Px = resetmap_par(B,x)
            Px = [1 0; 0 -B.e];
        end
    end
end