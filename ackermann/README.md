
# ROS2 Gazebo Ignition - Ackermann Steering Line Following Robot

## Creator:
Héctor Gordillo García

## Project Overview:
This project is based on the "Line Following Robot" designed by Pedro Arias, with several enhancements. It utilizes ROS2 with Gazebo and Ignition for simulating a robot with Ackermann steering that follows a line on the floor. The robot is controlled either autonomously by following a line using camera topics or manually via a PS4 controller.

### Key Features:
- **Ackermann Steering**: The robot now features Ackermann steering for better control and realism.
- **Line Following**: The robot autonomously follows a line on the map using camera topics.
- **PS4 Controller**: The robot can also be controlled manually using a PS4 controller.
- **Integration with ROS2**: The project integrates with ROS2, Gazebo, and Ignition for simulation and control.

## Installation:

1. **Install ROS2** (If not already installed):
   Follow the official ROS2 installation guide:  
   [ROS2 Installation](https://docs.ros.org/en/rolling/Installation.html)

2. **Install Gazebo and Ignition**:
   Make sure you have Gazebo and Ignition properly set up. You can follow these steps:
   - [Install Gazebo](http://gazebosim.org/)
   - [Install Ignition](https://ignitionrobotics.org/)

3. **Clone the Repository**:
   Clone this repository to your workspace:
   ```bash
   git clone https://github.com/yourusername/ackermann_line_following_robot.git
   ```

4. **Build the Workspace**:
   Navigate to your ROS2 workspace and build the project:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

5. **Source the Setup File**:
   Don't forget to source the workspace before running the simulation:
   ```bash
   source install/setup.bash
   ```

## Usage:

There are two main nodes in this project:

### 1. **Line Following Node** (`ackermann_follow.py`):
   - This node follows a line using the camera topics to detect the line on the map.
   - It is based on Pedro Arias' `f1_node.py`, with modifications to the PID control for Ackermann steering.

   To execute this node, run:
   ```bash
   python3 ackermann_follow.py
   ```

### 2. **PS4 Controller Node** (`ps4_node.py`):
   - This node allows manual control of the robot using a PS4 controller, which interacts with the Ackermann steering system.

   To execute this node, run:
   ```bash
   python3 ps4_node.py
   ```

## Acknowledgments:
- **Pedro Arias**: Original design of the Line Following Robot and the base node `f1_node.py`.
- **Héctor Gordillo García**: Modified the PID control, added Ackermann steering, and implemented the PS4 control node.

## License:
This project is licensed under the MIT License.
