# ROS 2 Robot Controller - Assignment 2

## Project Description
This assignment implements a control system for a mobile robot using **ROS 2**. The system is composed of two main nodes that communicate using Topics and Services.

The goal is to drive a robot around a circuit while automatically avoiding obstacles using Laser Scan data.

## Package Structure
The project contains two packages:

1.  **`assignment2_rt`**
    * Contains the C++ and Python source code.
    * Includes the Launch file to start the simulation.
2.  **`assignment2_custom_msgs`**
    * Defines the custom service used to calculate the robot's velocity.

##  Nodes Overview

### 1. Robot Controller Node (C++)
* **File:** `src/robot_controller.cpp`
* **Role:** The autonomous navigation node.
* **Functionality:**
    * **Subscribes** to the `/scan` topic to detect obstacles.
    * **Publishes** velocity commands to the `/cmd_vel` topic.
    * Implements a safety stop if obstacles are too close.

### 2. User Interface Node (Python)
* **File:** `scripts/user_interface.py`
* **Role:** The manual control node.
* **Functionality:**
    * Allows the user to start or stop the robot.
    * Modifies the **safety threshold** parameter.
    * Calls a **Service** to compute the robot's average velocity.

##  How to Run

### Step 1: Build
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2: Launch Simulation
```bash
ros2 launch assignment2_rt assignment2.launch.py
```
### Step 3: Run UI
Open a new terminal:
```bash
ros2 run assignment2_rt user_interface.py
```
