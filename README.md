# Robotics Technical Assessment

A ROS 2 based robotics technical assessment covering autonomous navigation, navigation monitoring, and robotic arm toolpath planning.

---

## Overview

This repository contains the implementation of three robotics tasks:

1. **Autonomous Mobile Robot Navigation**
2. **Navigation Monitoring**
3. **xArm6 Cone Toolpath Planning and Execution**

The projects are implemented using ROS 2 and related robotics tools, with simulation and visualization support.

---

# Task 1 — Autonomous Navigation

The first task focuses on autonomous navigation of a mobile robot in a simulated environment.

The implementation includes the robot description, simulation environment, localization, controllers, maps, navigation configuration and Nav2 integration.

### Technologies

- ROS 2 Humble
- Gazebo
- RViz2
- Nav2
- Python
- C++
- URDF / Xacro
- ROS 2 Control

### Features

- Mobile robot simulation
- Robot description using URDF/Xacro
- Differential-drive control
- Odometry
- TF transformations
- Localization
- Map-based navigation
- Global and local costmaps
- Path planning
- Navigation controller
- Goal-based autonomous navigation
- Dynamic obstacle simulation
- RViz visualization

### Package Structure

```text
task1_navigation/
│
├── bumperbot_description/
│   ├── models/
│   ├── urdf/
│   ├── worlds/
│   └── rviz/
│
├── bumperbot_localization/
│   ├── config/
│   ├── include/
│   ├── src/
│   └── launch/
│
├── bumperbot_bringup/
│   └── launch/
│
├── bumperbot_controller/
│   ├── config/
│   ├── include/
│   ├── src/
│   └── launch/
│
├── maps/
├── config/
├── launch/
├── rviz/
└── worlds/
Navigation Maps

The repository contains several maps used during development and testing:

assignment_map
navigation_map
turtlebot3_map
walls_only
empty_map
Navigation Configuration

The Nav2 configuration is located at:

task1_navigation/config/nav2_params.yaml
Task 2 — Navigation Monitor

The second task implements a ROS 2 navigation monitoring node.

The monitor observes the navigation system and provides information about the current navigation state.

Technologies
ROS 2 Humble
Python
rclpy
ROS 2 Actions
Navigation messages
Geometry messages
Monitored Information

The node can monitor:

Navigation goals
Goal status
Robot pose
Navigation progress
Goal completion
Navigation failures
Package
task2_nav_monitor/
└── nav_monitor/
    ├── nav_monitor/
    │   ├── __init__.py
    │   └── nav_monitor_node.py
    ├── package.xml
    ├── setup.py
    └── setup.cfg
Task 3 — xArm6 Cone Toolpath

The third task focuses on generating and executing a Cartesian toolpath for an xArm6 robotic arm.

The implementation uses MoveIt for motion planning and inverse kinematics validation.

Technologies
ROS 2 Humble
MoveIt 2
Python
xArm6
Cartesian path planning
Inverse kinematics
Features
Cone toolpath generation
Cartesian trajectory generation
Inverse kinematics testing
Workspace probing
IK diagnostics
Trajectory generation
Trajectory execution
CSV trajectory storage
Scripts
scripts/
├── generate_cone_path.py
├── generate_trajectory.py
├── solve_cone_ik.py
├── test_moveit_ik.py
├── test_cone_pose.py
├── diagnose_cone_ik.py
├── probe_workspace.py
└── execute_cone_trajectory.py
Generated Data

The generated trajectories and test results are stored as CSV files:

cone_cartesian_path.csv
cone_ik_test.csv
cone_toolpath.csv
Software Requirements

The project was developed and tested using:

Ubuntu 22.04
ROS 2 Humble
Python 3
Gazebo
RViz2
Nav2
MoveIt 2
CMake
Git
Workspace Setup

Clone the repository:

git clone https://github.com/anandgawai123456-glitch/internship-technical-assessment.git

Enter the repository:

cd internship-technical-assessment

Source ROS 2 Humble:

source /opt/ros/humble/setup.bash

Build the workspace:

colcon build --symlink-install

Source the workspace:

source install/setup.bash
Running the Projects
Task 1

Build and source the workspace:

colcon build --symlink-install
source install/setup.bash

Launch the required simulation and navigation components using the launch files provided in:

task1_navigation/

The navigation system can then be visualized and controlled through RViz2.

Task 2

Build the workspace:

colcon build --symlink-install
source install/setup.bash

Run the navigation monitor:

ros2 run nav_monitor nav_monitor_node

The node monitors the navigation system while navigation goals are being executed.

Task 3

Build and source the workspace:

colcon build --symlink-install
source install/setup.bash

The xArm6 package contains launch files and scripts for testing the generated cone trajectory.

The main launch file is:

task3_xarm6_cone_toolpath/
└── xarm6_cone_toolpath/
    └── launch/
        └── cone_demo.launch.py
Repository Structure
internship-technical-assessment/
│
├── task1_navigation/
│   ├── bumperbot_description/
│   ├── bumperbot_localization/
│   ├── bumperbot_bringup/
│   ├── bumperbot_controller/
│   ├── config/
│   ├── launch/
│   ├── maps/
│   ├── rviz/
│   └── worlds/
│
├── task2_nav_monitor/
│   └── nav_monitor/
│
├── task3_xarm6_cone_toolpath/
│   └── xarm6_cone_toolpath/
│       ├── launch/
│       ├── scripts/
│       ├── test/
│       └── *.csv
│
└── README.md
Demonstration Video

The demonstration video for the assessment is available here:

Google Drive — Technical Assessment Demonstration

Author

Anand Gawai

GitHub:
https://github.com/anandgawai123456-glitch

Repository

The complete source code is available at:

https://github.com/anandgawai123456-glitch/internship-technical-assessment

https://drive.google.com/drive/folders/1eDTWtdHiKA-JBSJRda-t8TLNrVZT6IpS
