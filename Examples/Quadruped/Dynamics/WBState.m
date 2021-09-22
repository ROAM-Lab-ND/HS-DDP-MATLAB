classdef WBState
% Whole body state
    properties
        pos = zeros(3,1);
        rpy = zeros(3,1);
        q = zeros(12 ,1);
        vel = zeros(3,1);
        rpyrate = zeros(3,1);
        qd = zeros(12, 1);
    end
end