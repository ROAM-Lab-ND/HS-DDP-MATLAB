classdef BoundingModeLib < handle
    properties
        lib % structure array of events
    end
    methods
        function L = BoundingModeLib()
            L.lib = repmat(struct('mode',[], 'ctact',[], 'name', [],'duration', []), 1, 4);
            L.lib(1).mode = 1;
            L.lib(1).ctact = [0 1];
            L.lib(1).name = 'BS';
            L.lib(1).duration = 0.08;
            
            L.lib(2).mode = 2;
            L.lib(2).ctact = [0 0];
            L.lib(2).name = 'F1';
            L.lib(2).duration = 0.1;
            
            L.lib(3).mode = 3;
            L.lib(3).ctact = [1 0];
            L.lib(3).name = 'FS';
            L.lib(3).duration = 0.08;
            
            L.lib(4).mode = 4;
            L.lib(4).ctact = [0 0];
            L.lib(4).name = 'F2';
            L.lib(4).duration = 0.1;
        end
        
        function Pn = getNextMode(L, P)
            i = P.mode;
            if i+1 == 4
                Pn = L.lib(4);
            else
                Pn = L.lib(mod(i+1, 4));
            end
        end
        
        function S = genModeSequence(L, P, n)
            % n: length of sequence
            % P: current phase
            S = repmat(struct('mode',[], 'ctact',[], 'name', [], 'duration', []), 1, n);
            S(1) = P;
            for i = 2:n
                S(i) = L.getNextMode(S(i-1));
            end
        end
    end
end