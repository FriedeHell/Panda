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

%% Task-Space Trajectories mit Inverse Kinematics

% Initialisierung
frankaSpaceWaypoints = [];
frankaTimepoints = [];
numWaypoints = 0;

% Eingabeaufforderung für die Waypoint-Datei
waypointFile = input('Enter the waypoint file name: ', 's');

% Waypoint-Datei lesen
waypoints = readlines(waypointFile);

% Waypoints in numerische Werte konvertieren
waypoints = str2num(waypoints);

% Anzahl der Waypoints
numWaypoints = length(waypoints);

% Trajektorie generieren
[pos,vel] = minjerkpolytraj(waypoints,frankaTimepoints,numWaypoints);

% IK-Solver initialisieren
rng(0) % Seed the RNG so the inverse kinematics solution is consistent
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;

% Gelenkpositionen berechnen
q = zeros(9,numWaypoints);
for i = 1:numWaypoints
    targetPose = trvec2tform(pos(:,i)')*eul2tform([0, 0, pi]);
    q(:,i) = ik("panda_hand",targetPose,weights,initialGuess);
    initialGuess = q(:,i); % Use the last result as the next initial guess
end

% Roboterarm anzeigen
figure
set(gcf,"Visible","on")
show(robot);

% Trajektorie ausführen
rc = rateControl(sampleRate);
for i = 1:numWaypoints
    show(robot, q(:,i),FastUpdate=true,PreservePlot=false);
    pause;
end