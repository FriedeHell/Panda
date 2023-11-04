

function initialGuess = generateInitialGuess(robot, robotToTCP)

    % Get the current TCP position
    currentTCPPosition = serialLinkRobot.getEndEffectorTransform('panda_hand').translation;

    % Calculate the desired TCP position
    desiredTCPPosition = robotToTCP.translation;

    % Calculate the error between the current TCP position and the desired TCP position
    error = desiredTCPPosition - currentTCPPosition;

    % Generate an initial guess for the joint angles
    initialGuess = robot.getHomeConfiguration();

    % Use the error to adjust the initial guess
    initialGuess = initialGuess + ikJacobian(robot, 'panda_hand')^-1 * error;

    % Move the initial guess closer to the current joint configuration
    initialGuess = initialGuess + 0.5 * (robot.getJointPositions() - initialGuess);

end