clc;
clear;

% Roboter als Variable importieren
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

% Load Calibration Data aus der Datei
load("CalibrationData.mat");

%% Hier sollten noch die Joint Position Limits eingesetzt werden. 


%% Globale Grafik für Simulation festlegen
