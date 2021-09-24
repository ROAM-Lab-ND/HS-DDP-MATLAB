classdef TouchdownConstraint < ConstraintAbstract
    properties 
        nctact
        robot
        params
        ground_height
        n
    end
    methods 
        function S = TouchdownConstraint(impact_foot)
            % impact_foot: 1x4 vector of impact status of each foot
            S.nctact = impact_foot;
            S.params = getMiniCheetahParams();
            S.robot = buildModel(S.params);       
            S.ground_height = 0;
            S.n = length(imp_foot_idx);
        end
        function t_array = compute(S, x)
            imp_foot_idx = find(S.nctact == 1);            
            t_array = repmat(TConstraintData(length(x)), 1, S.n);
            q = x(1:18, 1);            
            footIds = S.params.footIds;
            kneeLinkLength = S.params.kneeLinkLength;
            for i = 1:S.n
                foot = imp_foot_idx(i);
                p_foot = forward_kinematics(S.robot, q, footIds(foot), [0,0,-kneeLinkLength]');
                t_array(i).h = p_foot(3) - S.ground_height;
                funcName = sprintf('constr_par%d',foot);
                t_array(i).hx = (feval(funcName,q))';
            end                     
        end
        function params = get_params(S)
            % Initialize Augmented Lagrangian parameters
            param.sigma = 5;
            param.lambda = 0;
            params = repmat(param, 1, S.n);
        end
    end
end