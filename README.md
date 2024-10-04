Human-Robot Interaction for Behavior-Driven Service Robot
=========================================================

Table of Contents
-----------------

* [Introduction](#introduction)
* [Project Features](#project-features)
* [Dependencies](#dependencies)
* [Installation Instructions](#installation-instructions)
* [Running the Project](#running-the-project)
* [Commands to Run Specific Files](#commands-to-run)
* [Troubleshooting](#troubleshooting)

Introduction
------------

The project implements a multi-robot setup using TurtleBot3 robots in a Gazebo simulation environment. The robots navigate autonomously, interpret commands through speech recognition, and make decisions based on behavior trees. This system simulates real-world human-robot interactions, where multiple robots work in a shared environment, executing commands through natural language inputs.

The speech-to-command module was developed using a custom-trained SpaCy model on a multilingual dataset (English and Hindi) of 1.5 lakh text-label pairs. The model effectively converts spoken language into actionable commands for the robots, allowing for intuitive and efficient communication between humans and robots.

Project Features
----------------

* Multi-Robot Coordination: Multiple TurtleBot3 robots navigating autonomously using the Nav2 stack.
* Behavior Tree-based Decision Making: Robots use behavior trees for task execution and decision-making.
* Multilingual Speech-to-Command Execution: Spoken commands in English and Hindi are translated into text, which is then interpreted by the model to generate corresponding robot commands that are sent to ROS2 for execution.
* Gazebo Simulation: Full simulation of robot navigation and coordination in a virtual environment.

Dependencies
------------

Before running this project, ensure the following dependencies are installed:

### General Dependencies

* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo 11](http://gazebosim.org/tutorials?tut=install_gazebo&cat=install)

### Python Libraries

* speech_recognition: For speech-to-text conversion.
* pyttsx3: For text-to-speech conversion.
* spacy: For natural language processing and text classification.
* rclpy
* geometry_msgs

### TurtleBot3

TurtleBot3 needs to be installed, and the environment variables should be properly configured. Follow the official [TurtleBot3 setup guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

Installation Instructions
-------------------------

Follow these steps to set up the project:

### Step 1: Set Up the Workspace

    mkdir -p robot_ws/src
    cd robot_ws/src
    
    # Clone the repository (use the appropriate branch for Humble)
    git clone https://github.com/singhparjanya/turtlebot3_multi_robot.git
    
    cd robot_ws

### Step 2: Install Dependencies

Source the ROS2 environment and install dependencies using rosdep:

    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src -r -y

### Step 3: Build the Workspace

    cd robot_ws
    colcon build --symlink-install
    source ./install/setup.bash

Running the Project
-------------------

To start the multi-robot simulation, use the following command:

    ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable_drive:=False

This will launch the simulation in Gazebo. You will need to open three separate terminals for running the various components of the system:

### Terminal 1: Sending Commands

    cd robot_ws/src/turtlebot3_multi_robot/scripts
    ./send_command.py

### Terminal 2: Sending Goals

    cd robot_ws/src/turtlebot3_multi_robot/scripts
    ./send_goal.py

### Terminal 3: Running Behavior Tree (C++ Node)

    # Ensure the workspace is sourced
    source install/setup.bash
    ros2 run turtlebot3_multi_robot tree

Troubleshooting
---------------

* **Gazebo not launching:** Make sure you have sourced the ROS2 Humble environment and that Gazebo is correctly installed.
* **Multi-robot simulation lag:** Try reducing the simulation update rate or the number of robots in the simulation.
* **ROS2 node errors:** Ensure you have sourced the workspace before running any commands:

      source install/setup.bash

* **Missing Dependencies:** Ensure all dependencies are installed with:

      rosdep install --from-paths src -r -y
