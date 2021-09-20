classdef GRF_constraint < ConstraintAbstract
    properties
        ctact
    end
    methods 
        function Obj = GRF_constraint(contact)
           % ctact: 2x1 vector for contact status of front and back foot            
           Obj.ctact = contact;
        end
    end
    methods
        function p_array = compute(Obj, x, u, y)
            % return 1x3 structure array of path constraint
            ctact = Obj.ctact;
            mu_fric = 0.6; % static friction coefficient
            Cy = [0,    ctact(1),       0, ctact(2);
                -ctact(1)    ctact(1)*mu_fric, -ctact(2), ctact(2)*mu_fric;
                ctact(1)     ctact(1)*mu_fric, ctact(2),  ctact(2)*mu_fric];
            by = zeros(3,1);
            p_array = repmat(PathConstraintData(14,4,4), 1, 3);
            for i = 1:length(p_array)
                p_array(i).c = Cy(i,:) * y + by(i);
                p_array(i).cy = Cy(i,:)';
            end
        end
        function params = get_params(obj)
            % Initialize Reduced Barrier parameters            
            param = ReB_params_struct();
            param.delta = 0.1;
            param.delta_min = 0.01;
            param.eps = 0.01;
            params = repmat(param, 1, 3);
        end
    end
end