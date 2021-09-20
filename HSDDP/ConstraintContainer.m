classdef ConstraintContainer < handle
    properties
        pconstrObjs = []
        tconstrObjs = []
    end
    methods
        function C = ConstraintContainer()
        end        
    end
    methods 
        function add_pathConstraint(C, pconstrObj)
            C.pconstrObjs{end+1} = pconstrObj;
        end
        function p_array = compute_pConstraints(C, x, u, y)
            p_array = [];
            for i = 1:length(C.pconstrObjs)
                p_array = [p_array, C.pconstrObjs{i}.compute(x,u,y)];
            end
        end
        function params = get_reb_params(C)
            params = [];
            for i = 1:length(C.pconstrObjs)
                params = [params, C.pconstrObjs{i}.get_params()];
            end
        end
    end
    methods 
        function add_terminalConstraint(C, tconstrObj)
            C.tconstrObjs{end+1} = tconstrObj;
        end
        function t_array = compute_tConstraints(C, x)
            t_array = [];
            for i = 1:length(C.tconstrObjs)
                t_array = [t_array, C.tconstrObjs{i}.compute(x)];
            end
        end
        function params = get_al_params(C)
            params = [];
            for i = 1:length(C.tconstrObjs)
                params = [params, C.tconstrObjs{i}.get_params()];
            end
        end
    end
end