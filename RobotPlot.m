function robot = RobotPlot(ax, pose)
    scale = 0.05;
    t = pose(1:2);
    a = pose(3);
    R = [cos(a) -sin(a); sin(a) cos(a)];
    G = [R t(:); 0 0 scale];

    % RobotBody= G*[[[-0.2, -0.05,-0.2,0.5]; [-0.4,0,0.4,0]]; ones(1,4)];
    RobotBody= G*[[[0.2, 0.45,0.2,0.8]; [0.2,0.5,0.8,0.5]]; ones(1,4)];
    RobotColor = [0.066 0.918 0.053];

    robot = patch(ax, RobotBody(1,:), RobotBody(2,:), RobotColor);
end