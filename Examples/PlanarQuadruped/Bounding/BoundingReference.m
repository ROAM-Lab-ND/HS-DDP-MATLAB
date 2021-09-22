classdef BoundingReference < handle
    properties
        hybridR % structure array of trajectories
        GRF
    end
    methods
        function R = BoundingReference(hybridR_in)
            R.hybridR = hybridR_in;
            R.GRF = [0, 80.9521]';
        end
        
        function update(R, x0, problem)
            dt = problem.dt;
            vd = problem.vd;
            hd = problem.hd;
            assert(length(R.hybridR) == problem.nPhase, 'Hybrid trajectory of phase object array have unmatched dimension');
            wbstate{1} = [[0,-0.1432,-pi/25,0.35*pi,-0.65*pi,0.35*pi,-0.6*pi]';
                            vd;1;zeros(5,1)];
            wbstate{2} = [[0,-0.1418,pi/35,0.2*pi,-0.58*pi,0.25*pi,-0.7*pi]';
                            vd;-1;zeros(5,1)];
            wbstate{3} = [[0,-0.1325,-pi/40,0.33*pi,-0.48*pi,0.33*pi,-0.75*pi]';
                            vd;1;zeros(5,1)];
            wbstate{4} = [[0,-0.1490,-pi/25,0.35*pi,-0.7*pi,0.25*pi,-0.60*pi]';
                            vd;-1;zeros(5,1)];                        
            k = 1;
            for i = 1:length(R.hybridR)
                for j = 1:R.hybridR(i).len
                    R.hybridR(i).xd{j}(1) = x0(1) + (k - 1)*dt*vd;
                    R.hybridR(i).xd{j}(2) = hd;
                    if i <= problem.nWBPhase                        
                        R.hybridR(i).xd{j}(4:7) = [0.3*pi,-0.7*pi,0.3*pi,-0.7*pi]';
                        R.hybridR(i).xd{j}(8) = vd;
                        R.hybridR(i).yd{j} = [R.GRF;R.GRF];
                    else
                        R.hybridR(i).xd{j}(4) = vd;
                        R.hybridR(i).ud{j} =[R.GRF;R.GRF];
                    end    
                    if j < R.hybridR(i).len
                        k = k + 1;
                    end                    
                end
                if i <= problem.nWBPhase
                    R.hybridR(i).xd{end} = wbstate{problem.phaseSeq(i).mode};
                end
            end
        end
    end
end