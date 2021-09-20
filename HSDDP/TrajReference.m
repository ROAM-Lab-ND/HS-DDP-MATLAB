classdef TrajReference < handle
    properties
        xd
        ud
        yd
        len
    end
    methods 
        function R = TrajReference(xs, us, ys, len_horizon)
            R.xd = repmat({zeros(xs,1)}, 1, len_horizon);
            R.ud = repmat({zeros(us,1)}, 1, len_horizon-1);
            R.yd = repmat({zeros(ys,1)}, 1, len_horizon-1);
            R.len = len_horizon;
        end
    end
end