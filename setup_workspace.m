% Copyright (C) 2023 All rights reserved.
% Author:     Friedemann Heller
%
% Date:        Nov, 30 2023
%
% -------------------------------------------------
% setup_workspace
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested in Matlab 2023a

clc;
clear;

%% Import the robot
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

%% Load Calibration Data from File
load("CalibrationData.mat");

%% Standard Variables
robot_ip = '192.168.1.11'; %place your robot's ip!

q_init = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4];

O_T_EE_init = [0.707 -0.707 -0.0  0.3071;...
              -0.707 -0.707 -0.0 -0.0;...
              -0.0    0.0    -1    0.59;...
               0      0      0    1]';

collision_behavior = ... 
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0,... % lower_torque_thresholds_acceleration
             20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0,... % upper_torque_thresholds_acceleration
             20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0,... % lower_torque_thresholds_nominal
             20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0,... % upper_torque_thresholds_nominal
             20.0, 20.0, 20.0, 25.0, 25.0, 25.0,...       % lower_force_thresholds_acceleration	
             20.0, 20.0, 20.0, 25.0, 25.0, 25.0,...       % upper_force_thresholds_acceleration
             20.0, 20.0, 20.0, 25.0, 25.0, 25.0,...       % lower_force_thresholds_nominal
             20.0, 20.0, 20.0, 25.0, 25.0, 25.0];         % upper_force_thresholds_nominal

joint_impedance = [3000, 3000, 3000, 2500, 2500, 2000, 2000];
cartesian_impedance = [3000, 3000, 3000, 300, 300, 300];

EE_T_K = [1 0 0 0,...
          0 1 0 0,...
          0 0 1 0,...
          0 0 0 1];  %transformation from end effector frame to stiffness frame
      
F_T_EE =  [1 0 0 0,...
           0 1 0 0,...
           0 0 1 0,...
           0 0 0 1];  %transformation from flange to end effector frame

load_inertia = [0,...  %mass
                0,0,0,... %center of mass
                0.001,0,0,...
                0,0.001,0,...
                0,0,0.001];   % intertia matrix
            
%% Make assignments to base workspace
assignin('base','q_init',q_init);
assignin('base','O_T_EE_init',O_T_EE_init);
assignin('base','collision_behavior',collision_behavior);
assignin('base','joint_impedance',joint_impedance);
assignin('base','cartesian_impedance',cartesian_impedance);
assignin('base','EE_T_K',EE_T_K);
assignin('base','F_T_EE',F_T_EE);
assignin('base','load_inertia',load_inertia);