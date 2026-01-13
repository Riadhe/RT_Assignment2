# ROS 2 Robot Controller - Assignment 2

## Description
This project implements a ROS 2 controller for a mobile robot in Gazebo.
It includes:
1. A C++ Controller node for automatic obstacle avoidance (Safety).
2. A Python User Interface node for manual control and service calls.
3. Custom messages and services.

## How to Run
1. Launch the Simulation and Controller:
   `ros2 launch assignment2_rt assignment2.launch.py`
2. Run the User Interface:
   `ros2 run assignment2_rt user_interface.py`