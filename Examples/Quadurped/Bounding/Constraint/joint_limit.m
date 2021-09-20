classdef joint_limit < ConstraintAbstract
    methods 
        function Obj = joint_limit()
        end
    end
    methods
        function p_array = compute(Obj, x, u, y)
            % return 1x8 structure array of path constraint
            q = x(4:7);
            q = q(:);
            Cjoint = [-eye(4);
                      eye(4)];            
            bjoint =  [ pi/4;      % front hip min
                -0.1;      % front knee min
                1.15*pi;   % back hip min
                -0.1;      % back knee min
                pi;        % front hip max
                pi-0.2;    % front knee max
                0.1;       % back hip max
                pi-0.2];   % back knee max
            p_array = repmat(PathConstraintData(14,4,4), 1, 8);
            for i = 1:length(p_array)
                p_array(i).c = Cjoint(i,:) * q + bjoint(i);
                p_array(i).cx = [Cjoint(i,:), zeros(1,7)]';
            end
        end
        function params = get_params(Obj)
            % Initialize Reduced Barrier parameters
            param = ReB_params_struct();
            param.delta = 0.1;
            param.eps = 0;
            param.delta_min = 0.01;
            params = repmat(param, 1, 8);
        end
    end
end