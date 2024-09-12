# Human-Robot Interaction for Behavior-Driven Service Robot

## Introduction
This project implements a multi-robot system using TurtleBot3 robots in a ROS2 Humble simulation environment. The goal is to simulate real-world human-robot interactions where robots autonomously navigate, process commands, and make decisions based on behavior trees.

## Project Features
- Multi-Robot Coordination using TurtleBot3 robots
- Behavior Tree-based Decision Making
- Speech-to-Command Processing for human-robot interaction
- Simulated Gazebo environment

## Dependencies
- ROS2 Humble
- Gazebo 11
- `speech_recognition`: For speech-to-text conversion
- `pyttsx3`: For text-to-speech conversion
- `spacy`: For natural language processing and text classification
- `rclpy` and `geometry_msgs`

## Installation Instructions
1. **Create a workspace and clone the project:**
    ```bash
    mkdir -p robot_ws/src
    cd robot_ws/src
    git clone https://github.com/singhparjanya/turtlebot3_multi_robot.git
    cd robot_ws
    ```
2. **Install dependencies:**
    ```bash
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src -r -y
    ```
3. **Build the workspace:**
    ```bash
    colcon build --symlink-install
    source ./install/setup.bash
    ```

## Running the Project

To launch the multi-robot simulation in Gazebo:
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable_drive:=False
