classdef FootholdPlanner < handle
    properties
        model
        params
    end
    methods 
        function PL = FootholdPlanner(model)
            PL.model = model;
            mc3D_param = get3DMCParams();
            PL.params = get2DMCParams(mc3D_param);
        end
                       
        function foothold_prediction(PL, x, stanceTime, vd)
            xCoM = x(1);
            x_offset = vd * stanceTime/2;
            pfoot = zeros(4,1);
            pfoot(1) = xCoM + PL.params.hipLoc{1}(1) + x_offset;
            pfoot(2) = -0.404; 
            pfoot(3) = xCoM + PL.params.hipLoc{2}(1) + x_offset;
            pfoot(4) = -0.404;
            PL.model.set_foothold(pfoot);
        end
               
    end
end