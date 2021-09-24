function p = forward_kinematics(robot, q, linkidx, loc)
% Compute forward kinamatics
% Input robot: robot model built with spatial_v2
%       q: generalized coordinate
%       linkidx: link index
%       loc: location of point of interest in local frame
% return p: position of the point in base frame
q = q(:);
loc = loc(:);
T = eye(4);
for i = 1:linkidx
    [ XJ, ~ ] = jcalc( robot.jtype{i}, q(i) );
    Xup{i} = XJ * robot.Xtree{i};
    [R, r] = plux(Xup{i});
    R = R';
    T = T*[R, r; zeros(1,3), 1];
end
p = T * [loc;1];
p = p(1:3, 1);
end