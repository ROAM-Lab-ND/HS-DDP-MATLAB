classdef RCostPartialData
    properties
        lx
        lu
        ly
        lxx
        lux
        luu
        lyy
    end
    methods
        function lpar = RCostPartialData(xsize, usize, ysize)
            lpar.lx = zeros(xsize, 1);
            lpar.lu = zeros(usize, 1);
            lpar.ly = zeros(ysize, 1);
            lpar.lxx = zeros(xsize, xsize);
            lpar.lux = zeros(usize, xsize);
            lpar.luu = zeros(usize, usize);
            lpar.lyy = zeros(ysize, ysize);
        end
    end
end