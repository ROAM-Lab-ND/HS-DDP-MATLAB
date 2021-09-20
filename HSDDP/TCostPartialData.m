classdef TCostPartialData
    properties
        G
        H
    end
    methods
        function phi_par = TCostPartialData(xsize)
            phi_par.G = zeros(xsize, 1);
            phi_par.H = zeros(xsize, xsize);
        end
    end
end