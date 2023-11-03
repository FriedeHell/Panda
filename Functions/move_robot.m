function move_robot(robot, fixed_point, radius, num_samples)

    % Define the start and end points of the trajectory.
    tcp_start_position = fixed_point - radius * [1, 0, 0];
    tcp_end_position = fixed_point + radius * [1, 0, 0];

    % Calculate the trajectory.
    tcp_trajectory = semicircle_trajectory(tcp_start_position, tcp_end_position, num_samples);

    % Move the robot along the trajectory.
    movealongtrajectory(robot, tcp_trajectory);
end