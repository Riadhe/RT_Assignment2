# ROS 2 Robot Controller - Assignment 2

## Project Description
This assignment implements a control system for a mobile robot using **ROS 2**. The system is composed of two main nodes that communicate using Topics and Services.

The goal is to drive a robot around a circuit while automatically avoiding obstacles using Laser Scan data.

The project demonstrates key ROS 2 concepts:
* **Publish/Subscribe** communication for sensor data and motor control.
* **Service/Client** architecture for on-demand computations.
* **Launch Files** for system orchestration.

---

## Package Structure
The solution is organized into two separate packages:

### 1. `assignment2_rt` (Source Code)
This package contains the logic nodes and launch files.
* `src/robot_controller.cpp`: The C++ core node handling navigation and collision avoidance.
* `scripts/user_interface.py`: The Python node for user interaction.
* `launch/assignment2.launch.py`: A master launch file that starts Gazebo, Rviz, and the controller node simultaneously.

### 2. `assignment2_custom_msgs` (Interfaces)
This package defines the custom data structures used for communication.
* `srv/GetAvgVel.srv`: A custom service definition. It returns the calculated average linear and angular velocity of the robot over a specific time window.

---

##  Nodes and Logic Overview

### 1. Robot Controller Node (`robot_controller.cpp`)
This is the "brain" of the robot. It operates autonomously to keep the robot moving while ensuring safety.

* **Communication:**
    * **Subscribes to `/scan`**: Reads ranges from the laser scanner.
    * **Publishes to `/cmd_vel`**: Sends velocity commands to the robot.
    * **Service Server**: provides velocity data upon request.

* **The Logic (Algorithm):**
    1.  The node continuously analyzes the Laser Scan ranges.
    2.  It calculates the minimum distance to the nearest obstacle.
    3.  **Safety Check:** If the distance is lower than the `safety_threshold` (default: 1.0m), the robot forces a stop.
    4.  **Navigation:** If the path is clear, the robot proceeds with its default driving behavior.

### 2. User Interface Node (`user_interface.py`)
This node acts as a remote control console for the operator. It decouples the heavy processing (C++) from the user interaction (Python).

* **Communication:**
    * **Service Client:** Connects to the `GetAvgVel` service to request speed data.
    * **Parameter Client:** Sends requests to the Controller node to update the `safety_threshold` dynamically.

* **User Menu Features:**
    1.  **Enable/Disable Robot:** Starts or stops the autonomous driving behavior.
    2.  **Get Average Velocity:** Calls the custom service to display the robot's average speed.
    3.  **Set Safety Threshold:** Allows the user to change how close the robot can get to walls (e.g., change from 1.0m to 0.5m) in real-time.

---

##  Installation & Build

### Prerequisites
* ROS 2 (Humble or Foxy)
* Gazebo Simulator
* `bme_gazebo_sensors` package (for Laser simulation)

### Compiling the Project
Clone the repository into your workspace `src` folder and build the packages:

```bash
cd ~/ros2_ws/src
# (Clone your repository here)
cd ~/ros2_ws
# Build the custom interfaces first, then the logic
colcon build --packages-select assignment2_custom_msgs
colcon build --packages-select assignment2_rt
source install/setup.bash
```
## How to Run
### Step 1: Launch the Simulation
This command starts the environment, the robot model, and the autonomous controller.


```bash

ros2 launch assignment2_rt assignment2.launch.py
```

### Step 2: Run the User Interface
Open a second terminal to access the control menu.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run assignment2_rt user_interface.py
```
