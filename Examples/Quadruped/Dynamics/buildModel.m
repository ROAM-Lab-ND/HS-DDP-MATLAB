function robot = buildModel(params)
% This function creates a full planar paramsruped model structure
% The 0-configuration is with legs straight down, cheetah
% pointed along the +x axis of the ICS.
% The ICS has +z up, +x right, and +y inner page
% Planar model has 7 DoFs, x, z translation, rotation around y
% and front (back) hip and knee rotations

%% Fixed-base model
% We first creat a fixed-base model
robot.NB = 13;                                 % number of bodies 
robot.parent  = zeros(1,robot.NB);             % parent body indices
robot.Xtree   = repmat({eye(6)},robot.NB,1)';   % coordinate transforms
robot.jtype   = repmat({'  '},robot.NB,1)';     % joint types
robot.I       = repmat({zeros(6)},robot.NB,1)'; % spatial inertias
robot.gravity = [0 0 -9.81]';                  % gravity acceleration vec
robot.e       = 0;                             % restitution coefficient at impact

nbase = 1; % floating base index for attaching four children links (hip links)
robot.parent(nbase) = 0;
robot.Xtree{nbase} = eye(6);
robot.jtype{nbase} = ' '; % is not needed since floating-base model would not use
robot.I{nbase} =  mcI(params.bodyMass, params.bodyCoM, params.bodyRotInertia);

NLEGS = 4;
nb = nbase;
for i = 1:NLEGS
    % Abad link
    nb = nb + 1;
    robot.parent(nb) = nbase;
    robot.Xtree{nb} = plux(eye(3), params.abadLoc{i});
    robot.jtype{nb} = 'Rx';
    robot.I{nb} = mcI(params.abadLinkMass, params.abadLinkCoM, params.abadRotInertia);
    
    % Hip link
    nb = nb + 1;
    robot.parent(nb) = nb - 1; % parent of the hip link is abad link
    robot.Xtree{nb} = plux(rz(pi), params.hipLoc{i});  % translation (half the body length)
    robot.jtype{nb} = 'Ry';
    robot.I{nb} = mcI(params.hipLinkMass, params.hipLinkCoM, params.hipRotInertia);
    
    % Knee link
    nb = nb + 1;
    robot.parent(nb) = nb - 1; % parent of the knee link is hip link
    robot.Xtree{nb} = plux(eye(3), params.kneeLoc);    % translation (length of hip link)
    robot.jtype{nb} = 'Ry';
    robot.I{nb} = mcI(params.kneeLinkMass, params.kneeLinkCoM, params.kneeRotInertia);
end
robot = floatbase(robot);
end