clc;
clear;



% Import the robot
% % panda = importrobot('frankaEmikaPanda.urdf');
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

%% Hier sollten noch die Joint Position Limits eingesetzt werden. 
