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

