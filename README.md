# F1Tenth Wall-Following Control

## Overview
This project implements a wall-following control algorithm for the F1Tenth competition using ROS 2. The algorithm utilizes a PID controller to maintain a constant distance from the right wall based on LiDAR readings. The control node processes sensor data and computes steering commands for the vehicle.

## Features
- Uses a PID controller to follow the right wall
- Processes LiDAR scan data to compute the distance to the wall
- Publishes steering and velocity commands to the vehicle
- Keyboard control for manual speed adjustments
- Configurable parameters for tuning the control algorithm

## Project Structure
```
wall_following_control/
├── config/
│   ├── pid_config.yaml    # Configuration file for PID parameters and topics
├── launch/
│   ├── steering.launch.py # Launch file to start the steering control node
├── wall_following_control/
|   ├── __init__.py
│   ├── steering_controller.py  # Steering control node implementation
├── setup.cfg
├── setup.py
├── package.xml
├── README.md
```

## Installation
### Prerequisites
- ROS 2 installed
- F1Tenth Simulator setup

### Building the Package
```sh
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --packages-select wall_following_control
source install/setup.bash
```

## Running the Node
### Launching the Controller
```sh
ros2 launch wall_following_control steering.launch.py
```

### Manual Control (Optional)
- Press `w` to move forward
- Press `s` to move backward
- Press `b` to activate boost
- Release the key to stop

## Configuration
The `pid_config.yaml` file contains tunable parameters:
```yaml
steering_node:
  ros__parameters:
    scan_topic: "/scan"
    drive_topic: "/drive"
    is_right_wall: True
    speed: 2.2
    boost: 3.2
    reference_angle: -60.0
    goal_distance: 1.0
    max_steering: 0.75
    kp: 1.6
    kd: 2.25
    ki: 0.0
```
Modify these values to fine-tune the PID controller.

## How It Works
1. The `steering_controller.py` node subscribes to LiDAR scan data.
2. The reference angle is used to extract the distance to the right wall.
3. The PID controller computes the steering angle based on the error (goal distance - measured distance).
4. Steering commands are published to the `/drive` topic.

## Future Improvements
- Implement adaptive tuning for PID parameters.
- Add support for left-wall following.
- Improve handling of sharp turns and obstacles.

## License
This project is licensed under the MIT License.
