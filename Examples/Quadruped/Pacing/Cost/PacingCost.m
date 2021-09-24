classdef PacingCost < handle
    properties
        Q
        R
        S
        Qf
        dt
    end
    methods
        function C = PacingCost(dt)
            pos_s = 3;
            ori_s = 3;
            qa_s = 12;
            qs = 18;
            qds = qs;
            xs = 2 * qs;
            us = 12;
            ys = 12;
            
            C.dt = dt;
            C.Q = blkdiag( eye(pos_s), eye(ori_s), eye(qa_s), ...
                           eye(pos_s), eye(ori_s), eye(qa_s));
            C.R = eye(us) * 0.1;
            C.Qf = blkdiag( eye(pos_s), eye(ori_s), eye(qa_s), ...
                            eye(pos_s), eye(ori_s), eye(qa_s)) * 10;
            C.S = zeros(ys);
        end
        function l = running_cost(C,k,x,u,y,mode,r)
            l = wsquare(x - r.xd{k}, C.Q{mode});
            l = l + wsquare(u - r.ud{k}, C.R{mode});
            l = l + wsquare(y - r.yd{k}, C.S{mode});
            l = l * C.dt * 0.5;
        end
        function lpar = running_cost_partial(C, k, x, u, y, mode, r)
            lpar.lx    = C.dt * C.Q{mode} * (x-r.xd{k}); % column vec
            lpar.lu    = C.dt * C.R{mode} * (u-r.ud{k});
            lpar.ly    = C.dt * C.S{mode} * (y-r.yd{k});
            lpar.lxx   = C.dt * C.Q{mode};
            lpar.lux   = zeros(length(u),length(x));
            lpar.luu   = C.dt * C.R{mode};
            lpar.lyy   = C.dt * C.S{mode};
        end
    end
    methods
        function phi = terminal_cost(C, x, mode, r)
            phi = 0.5* wsquare(x - r.xd{end}, C.Qf{mode});
        end
        function phi_par = terminal_cost_partial(C, x, mode, r)
            phi_par.G = C.Qf{mode} * (x - r.xd{end});
            phi_par.H = C.Qf{mode};
        end
    end
end