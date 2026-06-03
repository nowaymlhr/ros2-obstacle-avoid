# Obstacle Avoidance Line Follower — ROS2 & Gazebo

## Overview
A vision and LiDAR-based robot that follows a black track while simultaneously detecting and avoiding obstacles in its path. Built in Gazebo Harmonic using ROS2 Jazzy.

## Demo
[Watch Demo]([https://youtube.com/your_link](https://youtu.be/stDQCfnTQoE))

## What's New vs Line Follower v1
The previous project only followed a line using a camera sensor. This version adds a 2D LiDAR sensor for real-time obstacle detection, and a decision-making node that switches between line following and obstacle avoidance behaviors using a state machine.

## Package Structure

### line_controller
Contains three nodes:
- `line_detection.py` — subscribes to `/camera/image_raw`, processes the image using grayscaling and thresholding, calculates line centroid via moments, publishes error to `/track_detection`
- `obstacle_avoidance.py` — subscribes to `/scan` (LaserScan), detects obstacles using 2D LiDAR data
- `decision_making.py` — subscribes to both `/track_detection` and `/scan`, runs a state machine to switch between line following and obstacle avoidance, publishes to `/cmd_vel`

### line_follower_bot
Contains URDF/Xacro files for the robot:
- `base.xacro` — robot chassis, links and joints
- `camera.xacro` — downward-facing camera sensor and Gazebo plugin
- `lidar.xacro` — 2D LiDAR sensor and Gazebo plugin
- `properties.xacro` — all dimensions and physical properties

### my_robot_bringup
Contains:
- Launch file — starts Gazebo, RViz, robot state publisher and all nodes
- World SDF — simulation environment with track and static obstacles
- `gazebo_bridge.yaml` — ROS-Gazebo topic bridge configuration

## Dependencies
- ROS2 Jazzy
- Gazebo Harmonic
- ros_gz_sim
- ros_gz_bridge
- cv_bridge
- OpenCV

## How it Works

### Line Following
1. Camera captures image of track below
2. Image converted to grayscale and thresholded to isolate black line
3. Centroid calculated via image moments
4. Error = centroid x - image center (320px)
5. PID controller converts error to steering commands

### Obstacle Avoidance
1. 2D LiDAR scans 360° horizontal plane
2. Front zone (indices 160-200) monitored for obstacles
3. When obstacle detected within 1.2m — state machine enters avoidance mode
4. Robot turns toward side with more clearance, moves forward to clear obstacle
5. Returns to line following mode automatically

## How to Run

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone git@github.com:yourusername/repo-name.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch
ros2 launch my_robot_bringup line_follower.launch.xml

# Start the robot (second terminal)
source ~/ros2_ws/install/setup.bash
ros2 param set /decision_making start True
```

## Limitations & Future Improvements
- Sharp turns occasionally cause the robot to lose the track
- Recovery behavior when line is completely lost could be more robust
- State machine could be extended with more sophisticated path planning
