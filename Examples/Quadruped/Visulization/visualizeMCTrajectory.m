function visualizeMCTrajectory(X)
robot = importrobot("mini_cheetah_mesh.urdf");
robot = float_base_quadruped(robot);
robot.DataFormat = 'column';

configs = X(1:18, :);

bigAnim = figure(198);
clf
set(bigAnim,'Renderer','OpenGL');
set(bigAnim, 'Position', get(0, 'Screensize'));

ax = show(robot, configs(:,1), "Frames","off","collision","off","PreservePlot",0);
set(ax, 'CameraPosition', [12, -12, 2.5]);
hold on
% drawFloor();
axis([-1 1 -1 1 -4 0.5])
% axis off
% box off
% axis equal

for config = configs
    set(ax, 'CameraTarget', config(1:3)');
    show(robot, config, "Frames","off",'collision','off','PreservePlot',0);
    pause(0.03);
end
end