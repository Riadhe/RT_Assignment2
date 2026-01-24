# ROS 2 Robot Controller - Assignment 2

**Author:** Riadh Bahri 

---

##  Project Description
This assignment implements a control system for a mobile robot using **ROS 2**. The system is composed of two main nodes that communicate using Topics and Services.

The goal is to drive a robot around a circuit while automatically avoiding obstacles using Laser Scan data.

The project demonstrates key ROS 2 concepts:
* **Publish/Subscribe** communication for sensor data and motor control.
* **Service/Client** architecture for on-demand computations.
* **Launch Files** for system orchestration.

---

### Key Features
* **Manual Control:** Drive the robot using keyboard commands .
* **Safety Override:** If the robot gets too close to an obstacle, it ignores user input and automatically reverses to a safe distance.
* **Safety Lock:** After an emergency backup, the robot "locks" its controls to prevent immediate collision loops. The user must explicitly acknowledge the stop to regain control.
* **Data Feedback:** Publishes the robot's state (distance, direction, threshold) to a custom topic.
* **Service Integration:** Calculates average velocity and allows dynamic updates to the safety threshold.

---

##  Architecture & Nodes

The system consists of two main nodes interacting via custom topics and services.

### 1. `user_interface.py` (Python)
This node handles user input and sends requests to the controller.
* **Publishes to:** `/user_cmd` (Not directly to `/cmd_vel`).
* **Service Client:** Calls `get_avg_vel` and `change_threshold`.
* **Functionality:**
    * Captures `W`, `A`, `S`, `D` keystrokes for driving.
    * Sends a "Stop" command when `X` is pressed (which resets the safety lock).

### 2. `robot_controller.cpp` (C++)
This node acts as the safety system.
* **Subscribes to:** `/user_cmd` (User input) and `/scan` (Laser).
* **Publishes to:** `/cmd_vel` (Actual wheels) and `/robot_state` (Custom info).
* **The Logic:**
    1.  **Normal Mode:** Forwards user commands from `/user_cmd` to `/cmd_vel`.
    2.  **Danger Mode:** If `distance < threshold`, it blocks user input and forces the robot to reverse.
    3.  **Wait Mode:** Once the robot reverses to a safe buffer distance (+0.5m), it locks movement. The user must press `x` (Stop) to unlock the system.

---

##  Custom Interfaces
The package uses the following custom definitions:

* **Message: `RobotState.msg`**
    * `float32 distance`: Distance to the closest obstacle.
    * `string direction`: Direction of the obstacle ("Front", "Left", "Right").
    * `float32 threshold`: The current safety setting.

* **Service: `GetAvgVel.srv`**
    * **Response:** Returns the average linear and angular velocity of the last 5 user commands.

* **Service: `ChangeThreshold.srv`**
    * **Request:** `float32 new_threshold` (e.g., 1.5).
    * **Response:** `bool success`.

---
##  Installation & Build

### Prerequisites
* ROS 2 (Jazzy)
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
## Controls (User Interface)

* e --> Move Forward
* d --> Move Backward
* s --> Turn Left
* f --> Turn Right
* x --> Stop OR Reset Safety Lock
* q --> Quit


**Note**: If the robot enters "Safety Mode" and backs up, it will stop and refuse to move. You must press X to acknowledge the stop before you can drive again.
