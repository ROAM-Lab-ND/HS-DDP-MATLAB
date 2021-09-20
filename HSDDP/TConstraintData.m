classdef TConstraintData
    properties
        h = 0
        hx = []       
        hxx = []        
    end  
    methods 
        function T = TConstraintData(xs)
            T.h = 0;
            T.hx = zeros(xs, 1);           
            T.hxx = zeros(xs, xs);         
        end
    end
end