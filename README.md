# F1Tenth

## Overview

This repository contains ROS2 packages designed for use with the F1Tenth project. The F1Tenth initiative provides a platform for designing, building, and racing autonomous 1/10th scale cars. This repository includes packages that implement various functionalities for controlling and simulating these vehicles.

## F1Tenth Gym Environment

The repository's packages were developed using the [F1Tenth Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros) environment. This environment provides a comprehensive setup for creating, testing, and simulating autonomous driving algorithms in a ROS2 framework, specifically for F1Tenth cars.

## Packages

This repository currently includes the following packages:

1. **gap_follow**: This package implements a gap-following algorithm for obstacle avoidance, allowing the vehicle to navigate around obstacles in its path.

2. **mpc**: This package contains a Model Predictive Control (MPC) algorithm for trajectory planning, optimizing the vehicle's path while adhering to constraints.

3. **pure_pursuit**: This package implements a Pure Pursuit algorithm for path tracking, enabling the vehicle to follow a predefined trajectory.

4. **rrt**: This package implements a Rapidly-exploring Random Tree (RRT) algorithm for path planning, helping the vehicle explore and navigate complex environments.

5. **safety_node**: This package monitors the vehicle's state and surroundings, ensuring safe operations and intervening if necessary.

6. **wall_follow**: This package implements a wall-following algorithm, allowing the vehicle to navigate by maintaining a consistent distance from walls or barriers.
