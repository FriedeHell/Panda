% Importieren des Robotermodells
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

% Abfrage, wie viele Punkte angefahren werden sollen
numPoints = input("Wie viele Punkte möchten Sie anfahren? ");

% Initialisieren der Variablen
points = [];
angles = [];

% Benutzereingaben abfragen
for i = 1:numPoints
    disp("Geben Sie den Abstand zum Punkt " + i + " ein: ");
    distance = inputdlg("");
    disp("Geben Sie den Winkel zum Punkt " + i + " um die x-Achse ein: ");
    xAngle = input("");
    disp("Geben Sie den Winkel zum Punkt " + i + " um die y-Achse ein: ");
    yAngle = input("");

    % Punkt hinzufügen
    points = [points [distance, xAngle, yAngle]];
    angles = [angles [xAngle, yAngle]];

    % Bestätigung abfragen
    disp("Punkt " + i + " hinzugefügt. Bestätigen Sie mit Enter.");
    pause;
end

% Winkel um 0 ergänzen
angles = [angles, zeros(size(angles, 1), 1)];

% Target Pose berechnen
%R = eul2rotm(angles(:,i));
%R = [R, zeros(2,1)];
%R = [R; zeros(1,3)];
%targetPose = tform2trvec(R);

targetPose = transform3d('t', points(:,i)', 'R', eul2rotm(angles(:,i)));

% Inverse Kinematik berechnen
q = inverseKinematics(RigidBodyTree, robot, 'panda_hand', targetPose);

% Roboter anzeigen
figure
set(gcf,"Visible","on")
show(robot);

% Roboterarm bewegen
for i = 1:numPoints
    show(robot, q(:,i),FastUpdate=true,PreservePlot=false);
    disp("Punkt " + i + " angefahren.");
    pause;
end