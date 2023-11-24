%% WORKING!!!

%% Defining Time vector for the trajectories
tpts = 0:4;
sampleRate = 20;
tvec = tpts(1):1/sampleRate:tpts(end);
numSamples = length(tvec);

%% Task-Space Trajectories mit Inverse Kinematics

% Define the specific point to move to
targetPose = trvec2tform([1, 0, 0])*eul2tform([0, 0, pi/2]);

%% Inverse Kinematics
rng(0) % Seed the RNG so the inverse kinematics solution is consistent
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;

% Define the weights for the inverse kinematics solver
weights = [0.2 0.2 0.2 1 1 1]; % Prioritize position over orientation

% Define the initial guess for the joint angles
initialGuess = [0, 0, 0, -pi/2, 0, 0, 0, 0.01, 0.01]'; % Choose an initial guess within the robot joint limits

% Calculate the inverse kinematics solution
q = ik("panda_hand",targetPose,weights,initialGuess);

%% Show the robot
show(robot, q,FastUpdate=true,PreservePlot=false);
