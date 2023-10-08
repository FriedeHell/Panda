clc;
clear;

% Import the robot
robot = loadrobot("frankaEmikaPanda", DataFormat="column");

% Manually set the home configuration for the Panda robot arm (excluding the gripper)
homeConfig = zeros(7, 1); % Modify this to set appropriate joint angles
initialGuess = homeConfig;

% Define the central point (target) that the robot will always point to
centralPoint = [0.5, 0, 0.35];

% Define the radius of the circular path
radius = 0.3;

% Number of waypoints
numWaypoints = 100;

% Generate waypoints on a circular path around the central point
theta = linspace(0, 2*pi, numWaypoints);
x = centralPoint(1) + radius * cos(theta);
y = centralPoint(2) + radius * sin(theta);
z = centralPoint(3) * ones(1, numWaypoints);
waypoints = [x; y; z];

% Initialize variables
q = zeros(7, numWaypoints);
weights = [0.2, 0.2, 0.2, 1, 1, 1]; % Prioritize position over orientation

% Inverse Kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;

% Calculate joint positions for each waypoint
for i = 1:numWaypoints
    % Define the target pose for the TCP
    targetPose = trvec2tform(waypoints(:, i)') * eul2tform([0, 0, pi]);

    % Perform inverse kinematics for the arm only
    solution = ik('panda_link8', targetPose, weights, initialGuess);
    
    % Extract the joint positions from the solution
    q(:, i) = solution(1:7); % Panda robot arm has 7 joints
    initialGuess = q(:, i); % Use the last result as the next initial guess
end

% Visualize the trajectory
figure
set(gcf, 'Visible', 'on')
show(robot, q(:, 1), 'Visuals', 'on', 'Collisions', 'off', 'PreservePlot', true);

% Create a rate control object for visualization
sampleRate = 20;
rc = rateControl(sampleRate);

% Display the robot moving along the trajectory
for i = 1:numWaypoints
    show(robot, q(:, i), 'Visuals', 'on', 'Collisions', 'off', 'PreservePlot', false);
    waitfor(rc);
end
