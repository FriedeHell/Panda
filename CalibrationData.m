%% Globale Variablen Koordinaten erstellen
global kalibrieren_x
global kalibrieren_y
global kalibrieren_z
global kalibrieren_abstand_TCP
global kalibrieren_abstand_messpunkt
global kalibrieren_abstand_sicherheit
global kalibrieren_winkel_TCP

% % Override Kalibrierung in der App. Hier können Werte auch manuell
% eingesetzt werden. (Kommentierung aufheben)
%     kalibrieren_x = 0;
%     kalibrieren_y = 0;
%     kalibrieren_z = 0;
%     kalibrieren_abstand_TCP = 0;
%     kalibrieren_abstand_messpunkt = 0;
%     kalibrieren_abstand_sicherheit = 0;
%     kalibrieren_winkel_TCP = 0;

%Varialben Initialisieren, falls Varialbe leer ist. (kalibrierdaten werden
%gespeichert)
if isempty(kalibrieren_x) || isempty(kalibrieren_y) || isempty(kalibrieren_z) || isempty(kalibrieren_abstand_TCP)...
        || isempty(kalibrieren_abstand_messpunkt) || isempty(kalibrieren_abstand_sicherheit) || isempty(kalibrieren_winkel_TCP)
    kalibrieren_x = 0;
    kalibrieren_y = 0;
    kalibrieren_z = 0;
    kalibrieren_abstand_TCP = 0;
    kalibrieren_abstand_messpunkt = 0;
    kalibrieren_abstand_sicherheit = 0;
    kalibrieren_winkel_TCP = 0;
end


%% Funktionen %%

% Kalibrierdaten aus dem Matlab Skript in die App laden
function loadCalibrationData(app, event)
frankaTimepoints = linspace(tvec(1),tvec(end),4);

    % Werte aus der globalen Variable lesen [mm]
    kalibrieren_x = get(globalVar('kalibriren_x'));
    kalibrieren_y = get(globalVar('kalibriren_y'));
    kalibrieren_z = get(globalVar('kalibriren_z'));

    kalibrieren_abstand_TCP = get(globalVar('kalibrieren_abstand_TCP'));
    kalibrieren_abstand_messpunkt = get(globalVar('kalibrieren_abstand_messpunkt'));
    kalibrieren_abstand_sicherheit = get(globalVar('kalibrieren_abstand_sicherheit'));
    kalibrieren_winkel_TCP = get (globalVar('kalibrieren_winkel_TCP'));

end


% Kalibrierdaten aus der App ins Skript speichern.
function saveCalibrationData(app, event)

    % Werte der Eingabefelder lesen
    x = get(app.KalibrierenXSpinner, 'Value');
    y = get(app.KalibrierenYSpinner, 'Value');
    z = get(app.KalibrierenZSpinner, 'Value');

    TCP = get(app.AbstandTCPSpinner, 'Value');
    messpunkt = get(app.AbstandRoboterMesspunktSpinner, 'Value');
    sicherheit = get(app.SicherheitsabstandSpinner, 'Value');
    winkel = get(app.WinkelHandSpinner, 'Value');


    % Werte der Eingabefelder in die globalen Variablen schreiben
    kalibrieren_x = x;
    kalibrieren_y = y;
    kalibrieren_z = z;

    kalibrieren_abstand_TCP = TCP;
    kalibrieren_abstand_messpunkt = messpunkt;
    kalibrieren_abstand_sicherheit = sicherheit;
    kalibrieren_winkel_TCP = winkel;

    % Globale Variablen in die .mat-Datei sichern. 
    save(fullfile('CalibrationData.mat'), 'kalibrieren_x', 'kalibrieren_y', ...
        'kalibrieren_z', 'kalibrieren_abstand_TCP', 'kalibrieren_abstand_messpunkt', ...
        "kalibrieren_abstand_sicherheit","kalibrieren_winkel_TCP");

end



