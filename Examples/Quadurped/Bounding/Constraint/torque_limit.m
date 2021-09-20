classdef torque_limit < ConstraintAbstract
    methods
        function Obj = torque_limit()
        end
    end
    methods 
        function p_array = compute(Obj, x, u, y)
            % Torque limit Cu*u + bu > = 0
            % return 1x8 structure array of path constraint
            us = length(u);
            p_array = repmat(PathConstraintData(14,4,4), 1, 2*us);
            Cu = [-eye(us);
                eye(us)];
            bu = 33*ones(2*us, 1);
            for i = 1:length(p_array)
                p_array(i).cu = Cu(i,:)';
                p_array(i).c = Cu(i,:) * u + bu(i);
            end
        end
        function params = get_params(Obj)
            % Initialize Reduced Barrier parameters            
            param = ReB_params_struct();
            param.delta = 0.1;
            param.eps = 0.01;
            param.delta_min = 0.01;
            params = repmat(param, 1, 8);
        end
    end
end