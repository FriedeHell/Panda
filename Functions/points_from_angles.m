function points_from_angles(fixpoint, horizontal_angle, vertical_angle)
    % Berechnen der x- und y-Koordinaten des gewünschten Punkts
    x = fixpoint(1) + horizontal_angle * cos(fixpoint(3));
    y = fixpoint(2) + horizontal_angle * sin(fixpoint(3));

    % Berechnen der z-Koordinate des gewünschten Punkts
    z = fixpoint(3) + vertical_angle;

    % Erstellen der Liste mit den gewünschten Punkten
    points = [x y z];

    return points;
end