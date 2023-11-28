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
% Create a figure

figure;

% Create a random vector of points
x = rand(100, 1);
y = rand(100, 1);

% Plot the points
plot(x, y, 'o');

% Add a title and axis labels
title('Demo Plot');
xlabel('x');
ylabel('y');

% Display the plot
grid on;

