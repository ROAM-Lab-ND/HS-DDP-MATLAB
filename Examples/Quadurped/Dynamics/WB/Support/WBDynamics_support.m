function WBDynamics_support(quadruped)
% This function converts symbolic expression to function handles and only
% executes once.
qsize = 7;
xsize = 14;
usize = 4;
ysize = 4;

q   = sym('q', [qsize, 1]); 
qd  = sym('qd', [qsize, 1]); 
x   = [q;qd];
u   = sym('u', [usize, 1]); 

% [mc, ~] = build2DminiCheetah();

% Jacobian and Jacobian derivative
p = sym('p', 2);

for linkidx = 1:5
    contactPos = quadruped.getPosition(q,linkidx, p);
    J = jacobian(contactPos, q);
    Jd = reshape(jacobian(reshape(J, [numel(J),1]),q)*qd, 2, qsize);
    % J and Jd partial w.r.t. x
    Jx  = MatrixPartial(J, x);
    Jdx = MatrixPartial(Jd, x);
    JFilename = sprintf('Kinematics/Link%dJacobian', linkidx);
    JPFilename = sprintf('Kinematics/Link%dJacobian_par', linkidx);
    matlabFunction(J, Jd,'file',JFilename,'vars',{x,p});
    matlabFunction(Jx, Jdx, 'file',JPFilename,'vars',{x,p});
end

% Dynamics partials
[H, C] = HandC(quadruped.model,q,qd);
Hx = MatrixPartial(H, x);
Cx = jacobian(C, x);

matlabFunction(Hx, Cx, 'file','Dynamics/WB/FreeDynamics_par','vars',{x});

fprintf('WBDynamics support functions generated successfully! \n');
end