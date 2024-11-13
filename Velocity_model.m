robot_pos = [0, 0, 0]; % Initial position [x, y, theta]
dt = 0.1; % Time step
goal = [10, 10]; % Goal position

obstacles = [3, 4; 5, 5; 8, 7]; % Obstacle positions
safe_distance = 1.5; % Safe distance to avoid obstacles
wheel_base = 0.5; % Wheelbase of the robot
v_max = 0.5; % Maximum forward speed

figure;
axis equal;
hold on;
plot(goal(1), goal(2), "rx", "MarkerSize", 10, "LineWidth", 2); % Plot goal
plot(obstacles(:, 1), obstacles(:, 2), "ko", "MarkerSize", 10, "LineWidth", 2); % Plot obstacles
xlim([-1, 11]);
ylim([-1, 11]);
xlabel("X Position");
ylabel("Y Position");
title("Velocity-based Navigation Algorithm");

while norm(robot_pos(1:2) - goal(1:2)) > 0.2
    % Calculate direction to goal
    direction_to_goal = atan2(goal(2) - robot_pos(2), goal(1) - robot_pos(1));
    error_angle = direction_to_goal - robot_pos(3);

    % Normalize error_angle to be between -pi and pi
    error_angle = atan2(sin(error_angle), cos(error_angle));

    % Calculate desired velocities based on error angle
    k_angle = 1.0; % Proportional gain for steering
    angular_velocity = k_angle * error_angle; % Calculate angular velocity based on error angle

    % Set base velocities for moving toward the goal
    v_left = v_max - angular_velocity * wheel_base / 2; % Adjust left wheel speed
    v_right = v_max + angular_velocity * wheel_base / 2; % Adjust right wheel speed

    % Ensure velocities are not negative
    v_left = max(0, v_left);
    v_right = max(0, v_right);

    % Check for obstacles and adjust velocities accordingly
    for i = 1:size(obstacles, 1)
        distance_to_obstacle = norm(robot_pos(1:2) - obstacles(i, :));
        if distance_to_obstacle < safe_distance
            % Calculate direction to the obstacle
            direction_to_obstacle = atan2(obstacles(i, 2) - robot_pos(2), obstacles(i, 1) - robot_pos(1));
            angle_diff = direction_to_obstacle - robot_pos(3);
            angle_diff = atan2(sin(angle_diff), cos(angle_diff)); % Normalize

            % If facing the obstacle directly, plan to turn
            if abs(angle_diff) < pi / 4 % Facing the obstacle within a threshold
                if angle_diff > 0
                    v_right = min(v_right, 0.2); % Reduce right wheel speed to turn left
                else
                    v_left = min(v_left, 0.2); % Reduce left wheel speed to turn right
                end
            end
        end
    end

    % Update robot's pose based on the velocities
    robot_pos(3) = robot_pos(3) + (v_right - v_left) / wheel_base * dt; % Update orientation
    robot_pos(1) = robot_pos(1) + (v_left + v_right) / 2 * cos(robot_pos(3)) * dt; % Update x position
    robot_pos(2) = robot_pos(2) + (v_left + v_right) / 2 * sin(robot_pos(3)) * dt; % Update y position

    % Plot the robot's position
    plot(robot_pos(1), robot_pos(2), "bo", "MarkerSize", 6, "LineWidth", 2);
    drawnow;

    % Display velocities in the console
    fprintf("Linear Velocity: %.2f m/s, Angular Velocity: %.2f rad/s\n", (v_left + v_right) / 2, (v_right - v_left) / wheel_base);

    pause(dt);
end

disp("Goal reached!");