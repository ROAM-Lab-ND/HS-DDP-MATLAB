function [Jv, Jw] = compute_Jacobian(model, q, linkidx, loc)
% Compute the Jacobian of a task-space point in base frame
% Input  q: generalized coordinate   q = [pos; rpy; qa];
%        linkidx: index of the link needs Jacobian computation
%        loc: location of the point in local frame of linkidx
% return Jv: Linear velocity Jacobian in base frame
%        Jw: Angular velocity Jacobian in base frame
import casadi.SX

q = q(:);
loc = loc(:);
if isa(q, 'numeric')
    J = zeros(6, model.NB);
end
if isa(q, 'SX')
    J = SX(6, model.NB);
end
if isa(q, 'sym')
    J = sym('J', [6, model.NB]);
end

X = cell(1, model.NB);
S = cell(1, model.NB);
for i = linkidx:-1:1
    [XJ, Si] = jcalc( model.jtype{i}, q(i));
    S{i} = Si;
    if i == linkidx
        Xup = XJ * xlt(loc);
        X{i} = Xup;
    else
        Xup = XJ * model.Xtree{i+1};
        X{i} = X{i+1} * Xup;
    end
    J(:,i) = X{i} * S{i};
end
[R, ~] = plux(X{linkidx});  % Orientation of base w.r.t. link frame
J = blkdiag(R', R') * J;    % Need transpose J to get rotation of link frame w.r.t. base frame
Jv = J(4:end, :);
Jw = J(1:3, :);
end