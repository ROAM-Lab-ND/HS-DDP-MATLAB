classdef PathConstraintData
    properties
        c = 0
        cx = []
        cu = []
        cy = []
        cxx = []
        cuu = []
        cyy = []
    end  
    methods 
        function C = PathConstraintData(xs,us,ys)
            C.c = 0;
            C.cx = zeros(xs, 1);
            C.cu = zeros(us, 1);
            C.cy = zeros(ys, 1);
            C.cxx = zeros(xs, xs);
            C.cuu = zeros(us, us);
            C.cyy = zeros(ys, ys);
        end
    end
end