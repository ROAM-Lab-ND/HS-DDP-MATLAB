classdef SwitchingConstraint < ConstraintAbstract
    properties 
        nctact
    end
    methods 
        function S = SwitchingConstraint(contact_next)
            S.nctact = contact_next;
        end
        function t_array = compute(S, x)
            t_array = TConstraintData(length(x));
            if isequal(S.nctact, [1, 0])
                [t_array.h, t_array.hx, t_array.hxx] = Front_TouchDown_Constraint(x);
                t_array.hx = t_array.hx(:);
            end
            if isequal(S.nctact, [0, 1])
                [t_array.h, t_array.hx, t_array.hxx] = Back_TouchDown_Constraint(x);
                t_array.hx = t_array.hx(:);
            end            
        end
        function params = get_params(S)
            % Initialize Augmented Lagrangian parameters
            param.sigma = 5;
            param.lambda = 0;
            params = [param];
        end
    end
end