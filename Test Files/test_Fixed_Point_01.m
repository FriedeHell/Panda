% Copyright (C) 2023 All rights reserved.
% Author:     Friedemann Heller
%
% Date:        Nov, 30 2023
%
% -------------------------------------------------
% CalibrationData
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested in Matlab 2023a

tcpDistance = 0.25;

tcpPosition = [0.25, 0, 0.25];

horizontalAngle =0;
verticalAngle = 0;

%robotToTCP = trvec2tform([tcpDistance, 0, 0]) * eul2tform([0, horizontalAngle, verticalAngle]);
robotToTCP = trvec2tform(tcpPosition) * eul2tform([0, horizontalAngle, verticalAngle]);

rng(0);
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [0.2 0.2 0.2 1 1 1]; % Prioritize position over orientation

% Convert the rigidBodyTree robot to a SerialLink robot
%serialLinkRobot = serialL

%initialGuess = [0, 0, 0, -pi/2, 0, 0, 0, 0.0, 0.0]'; 
initialGuess = generateInitialGuess(robot, robotToTCP); 
jointAngles = ik("panda_hand", robotToTCP, weights, initialGuess);

show(robot, jointAngles,FastUpdate=true,PreservePlot=false);