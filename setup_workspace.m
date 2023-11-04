clc;
clear;

%Adding Path for Franka Matlab ins Dokument. Keine Ahnung, ob ich das
%später auch noch brauche. Aber ich lass es mal im Startup-file mit drin.
%addpath("/home/hsaalen/franka_matlab_v0.3.1");


% Import the robot
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

% Load Calibration Data aus der Datei
load("CalibrationData.mat");

%% Hier sollten noch die Joint Position Limits eingesetzt werden. 


%% Globale Grafik für Simulation festlegen
