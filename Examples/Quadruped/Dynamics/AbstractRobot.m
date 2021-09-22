classdef  AbstractRobot < handle
    properties (Abstract,SetAccess=private, GetAccess=public)
        dt             
    end
    
    methods 
       % All member functions below implement nothing
        [x_next, y]     = dynamics(x,u) 
        dynInfo         = dynamics_par(x,u)
        [x_next, y]     = resetmap(x) 
        Px              = resetmap_par(x)         
        Initialize_model(varargin)                        
    end
end