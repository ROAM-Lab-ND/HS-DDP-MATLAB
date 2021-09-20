classdef ConstraintAbstract < handle
    methods(Abstract)
        p_array = compute(Obj,x,u,y)
        params = get_params(Obj)                
    end
end