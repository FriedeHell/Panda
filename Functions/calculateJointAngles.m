tcpDistance = 0.25;

tcpPosition = [0.25, 0, 0.25];

horizontalAngle = 30;
verticalAngle = 100;

robotToTCP = trvec2tform(tcpPosition) * eul2tform([0, horizontalAngle, verticalAngle]);

rng(0);
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [0.2 0.2 0.2 1 1 1]; % Prioritize position over orientation

initialGuess = [0, 0, 0, -pi/2, 0, 0, 0, 0.01, 0.01]';

jointAngles = ik("panda_hand", robotToTCP, weights, initialGuess);

show(robot, jointAngles,FastUpdate=true,PreservePlot=false);