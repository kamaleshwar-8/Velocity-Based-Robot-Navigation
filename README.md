# Velocity-Based Robot Navigation Algorithm

## Overview
This MATLAB script implements a velocity-based navigation algorithm for a differential drive robot. The robot autonomously navigates from a starting position to a goal position while avoiding obstacles using a reactive control strategy.

## Features
- Differential drive robot navigation
- Obstacle avoidance with safety distance
- Real-time visualization of robot's path
- Velocity control based on goal direction and obstacle proximity
- Live display of linear and angular velocities

## Prerequisites
- MATLAB (any recent version)
- MATLAB Figure Window support

## Parameters
- `robot_pos`: Initial robot position [x, y, theta]
- `dt`: Time step (0.1 seconds)
- `goal`: Target position [x, y]
- `obstacles`: Array of obstacle positions
- `safe_distance`: Minimum distance to maintain from obstacles (1.5 units)
- `wheel_base`: Robot's wheel base (0.5 units)
- `v_max`: Maximum forward velocity (0.5 units/s)

## How It Works
1. The robot continuously calculates the direction to the goal
2. Adjusts wheel velocities based on:
   - Angular error to goal direction
   - Proximity to obstacles
   - Safety constraints
3. Uses differential drive kinematics to update position
4. Implements obstacle avoidance by reducing wheel speeds when near obstacles
5. Continues until reaching within 0.2 units of the goal

## Visualization
The script provides real-time visualization showing:
- Robot's current position (blue dots)
- Goal position (red x)
- Obstacles (black circles)
- Complete path taken by the robot

## Output
- Real-time plot of robot's trajectory
- Console output of linear and angular velocities
- "Goal reached!" message upon completion
![vel_model](https://github.com/user-attachments/assets/7363cc4e-fdfd-4527-a824-93e525849378)

## Usage
1. Set the desired parameters in the script:
   ```matlab
   robot_pos = [0, 0, 0];  % Starting position
   goal = [10, 10];        % Goal position
   obstacles = [3, 4; 5, 5; 8, 7];  % Obstacle positions
   ```
2. Run the script
3. Watch the visualization as the robot navigates to the goal

## Notes
- The robot uses a proportional controller for steering
- Obstacle avoidance is achieved through reactive velocity adjustment
- The algorithm ensures non-negative wheel velocities
- The simulation runs until the robot is within 0.2 units of the goal
