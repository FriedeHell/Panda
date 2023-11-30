% Copyright (C) 2023 All rights reserved.
% Author:     Friedemann Heller
%
% Date:        Nov, 30 2023
%
% -------------------------------------------------
% VectorMode
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested in Matlab 2023a

%% Setup Measuring Point
measuringPoint = [1 0 1];

%% Definieren der ZeitvektorenW
tpts = 0:5;
sampleRate = 20;
tvec = tpts(1):1/sampleRate:tpts(end);
numSamples = length(tvec);

%% Task-Space Trajectories mit Inverse Kinematics
frankaSpaceWaypoints = [0.5 0.25 0.25; 0.5 0 0.35; 0.5 -0.25 0.25; 0.5 0.25 0.25]';

for i = 1:numSamples
    frankaSpaceWaypoints = [frankaSpaceWaypoints; [measuringPoint(1) frankaSpaceWaypoints(i,2) measuringPoint(3) 1]];
end
frankaTimepoints = linspace(tvec(1),tvec(end),numSamples);
[pos,vel] = minjerkpolytraj(frankaSpaceWaypoints,frankaTimepoints,numSamples);

%% Inverse Kinematics
rng(0) % Seed the RNG so the inverse kinematics solution is consistent
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;
q = zeros(9,numSamples);
weights = [0.2 0.2 0.2 1 1 1]; % Prioritize position over orientation
initialGuess = [0, 0, 0, -pi/2, 0, 0, 0, 0.01, 0.01]'; % Choose an initial guess within the robot joint limits
for i = 1:size(pos,2)
    targetPose = trvec2tform(pos(:,i)')*eul2tform([0, 0, measuringPoint(3)]);
    q(:,i) = ik("panda_hand",targetPose,weights,initialGuess);
    initialGuess = q(:,i); % Use the last result as the next initial guess
end

%% Show Figure
figure
set(gcf,"Visible","on")
show(robot);

%% If-Schleife zur Darstellung des Roboters auf der Bahn
rc = rateControl(sampleRate); % Stellt sicher, dass Roboter in Abtastfrequenz dargestellt wird. 
for i = 1:numSamples
    show(robot, q(:,i),FastUpdate=true,PreservePlot=false);
    waitfor(rc);
end

%% Move Robot
% Define Speed Factor (Between 0 and 1)
speed_factor = 0.5;

% Move the robot to each waypoint using joint angles
for i = 1:numSamples
    % Define the target joint configuration
    target_q = q(:, i);

    % Move the robot to the target configuration
    franka_joint_point_to_point_motion(robot_ip, target_q, speed_factor);

    pause(4);
end
