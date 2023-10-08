clc;
clear;

% Import the robot Visualization
panda = importrobot('frankaEmikaPanda.urdf');

%% Hier sollten noch die Joint Position Limits eingesetzt werden. 

% Show the robot
show(panda, 'Visuals', 'on', 'Collisions', 'off');

