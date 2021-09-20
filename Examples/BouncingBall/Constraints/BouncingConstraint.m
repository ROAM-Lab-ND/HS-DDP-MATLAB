classdef BouncingConstraint < ConstraintAbstract
    properties
        mode
    end
    methods
        function S = BouncingConstraint(mode)
            S.mode = mode;
        end
        function t_array = compute(S, x)
            t_array = TConstraintData(length(x));
            if S.mode == 1
                t_array.h = x(1);
                t_array.hx = [1 0]';
            else
                t_array.h = x(1) - 0.3;
                t_array.hx = [1 0]';
            end            
        end
        function params = get_params(S)
            % Initialize Augmented Lagrangian parameters
            param.sigma = 0.5;
            param.lambda = 0;
            params = [param];
        end
    end
end