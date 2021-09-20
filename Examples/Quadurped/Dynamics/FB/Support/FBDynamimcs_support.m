function FBDynamimcs_support(FBMC2D)
q_size = 3; %(x,z,pitch (theta))
x_size = 6;
u_size = 4; % ground reaction force F1 F2

q = sym('q',[q_size 1],'real'); % x, z, pitch
qd = sym('qd', [q_size 1], 'real');
x = [q; qd];
u = sym('u',[u_size 1],'real'); % [F1; F2]
p = sym('p',[4,1],'real'); % foothold location [p1; p2]
s = sym('s', [2,1], 'real'); % contact state

I = FBMC2D.Inertia;
m = FBMC2D.mass;

% continuous dynamics
f = [qd;
     s(1)*u(1:2,1)/m + s(2)*u(3:4)/m + [0,-9.81]';
     s(1)*(I\cross2D((p(1:2,1) - x(1:2,1)),u(1:2,1))) + s(2)*(I\cross2D((p(3:4,1) - x(1:2,1)),u(3:4,1)))];

% partials of continuous dynamics       
fx = jacobian(f, x);
fu = jacobian(f, u);

matlabFunction(f, 'file', 'Dynamics/FB/Support/FBDynamics','vars',{x,u,p,s});
matlabFunction(fx, fu, 'file','Dynamics/FB/Support/FBDynamics_par','vars',{x,u,p,s});

fprintf('Trunk Dynamics support funtions generated successfully!\n');
end


function result = cross2D(p1,p2)
p1_3D = [p1(1),0,p1(2)]';
p2_3D = [p2(1),0,p2(2)]';
y = [0 1 0]';
result = dot(y, cross(p1_3D,p2_3D));
end