function helperRobotMove(path, robot, scenario)
    for config = path'
        robot.Manipulator.JointPositions = config;
        update(scenario);
        pause(0.1); % Pause zwischen den Bewegungen
    end
end
