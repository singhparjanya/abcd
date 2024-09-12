<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <title>Human-Robot Interaction for Behavior-Driven Service Robot</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            line-height: 1.6;
            background-color: #f4f4f4;
            margin: 0;
            padding: 0;
        }
        .container {
            width: 80%;
            margin: auto;
            overflow: hidden;
        }
        header {
            background: #333;
            color: #fff;
            padding-top: 30px;
            min-height: 70px;
            border-bottom: #77aaff 3px solid;
        }
        header h1 {
            text-align: center;
            text-transform: uppercase;
            margin: 0;
            font-size: 2em;
        }
        nav {
            background: #555;
            color: #fff;
            padding: 10px 0;
            text-align: center;
        }
        nav a {
            color: #fff;
            padding: 10px;
            text-decoration: none;
            text-transform: uppercase;
        }
        section {
            padding: 20px;
            background: #fff;
            margin-bottom: 10px;
            border: 1px solid #ddd;
        }
        footer {
            background: #333;
            color: #fff;
            text-align: center;
            padding: 10px;
            margin-top: 20px;
            border-top: #77aaff 3px solid;
        }
    </style>
</head>
<body>
    <header>
        <div class="container">
            <h1>Human-Robot Interaction for Behavior-Driven Service Robot</h1>
        </div>
    </header>
    <nav>
        <a href="#intro">Introduction</a>
        <a href="#features">Features</a>
        <a href="#dependencies">Dependencies</a>
        <a href="#install">Installation</a>
        <a href="#commands">Commands</a>
        <a href="#troubleshoot">Troubleshooting</a>
    </nav>
    <div class="container">
        <section id="intro">
            <h2>Introduction</h2>
            <p>This project implements a multi-robot system using TurtleBot3 robots in a ROS2 Humble simulation environment. The goal is to simulate real-world human-robot interactions where robots autonomously navigate, process commands, and make decisions based on behavior trees.</p>
        </section>

        <section id="features">
            <h2>Project Features</h2>
            <ul>
                <li>Multi-Robot Coordination using TurtleBot3 robots</li>
                <li>Behavior Tree-based Decision Making</li>
                <li>Speech-to-Command Processing for human-robot interaction</li>
                <li>Simulated Gazebo environment</li>
            </ul>
        </section>

        <section id="dependencies">
            <h2>Dependencies</h2>
            <ul>
                <li>ROS2 Humble</li>
                <li>Gazebo 11</li>
                <li><code>speech_recognition</code>: Speech-to-text conversion</li>
                <li><code>pyttsx3</code>: Text-to-speech conversion</li>
                <li><code>spacy</code>: Natural language processing</li>
                <li><code>rclpy</code> and <code>geometry_msgs</code></li>
            </ul>
        </section>

        <section id="install">
            <h2>Installation Instructions</h2>
            <ol>
                <li>Create a workspace and clone the project:
                    <pre><code>mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/singhparjanya/turtlebot3_multi_robot.git
cd robot_ws</code></pre>
                </li>
                <li>Install dependencies:
                    <pre><code>source /opt/ros/humble/setup.bash
rosdep install --from-paths src -r -y</code></pre>
                </li>
                <li>Build the workspace:
                    <pre><code>colcon build --symlink-install
source ./install/setup.bash</code></pre>
                </li>
            </ol>
        </section>

        <section id="commands">
            <h2>Running the Project</h2>
            <p>To launch the multi-robot simulation in Gazebo:</p>
            <pre><code>ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable_drive:=False</code></pre>
            <p>Then, open three separate terminals:</p>
            <p><strong>Terminal 1: Send Command</strong></p>
            <pre><code>cd robot_ws/src/turtlebot3_multi_robot/scripts
./send_command.py</code></pre>
            <p><strong>Terminal 2: Send Goal</strong></p>
            <pre><code>cd robot_ws/src/turtlebot3_multi_robot/scripts
./send_goal.py</code></pre>
            <p><strong>Terminal 3: Behavior Tree (C++)</strong></p>
            <pre><code>source install/setup.bash
ros2 run turtlebot3_multi_robot tree</code></pre>
        </section>

        <section id="troubleshoot">
            <h2>Troubleshooting</h2>
            <ul>
                <li><strong>Gazebo not launching:</strong> Make sure you have sourced the ROS2 Humble environment.</li>
                <li><strong>Multi-robot simulation lag:</strong> Reduce simulation update rate or the number of robots.</li>
                <li><strong>ROS2 node errors:</strong> Ensure the workspace is sourced before running commands.</li>
                <li><strong>Missing dependencies:</strong> Install missing dependencies using:
                    <pre><code>rosdep install --from-paths src -r -y</code></pre>
                </li>
            </ul>
        </section>
    </div>

    <footer>
        <p>Human-Robot Interaction for Behavior-Driven Service Robot &copy; 2024</p>
    </footer>
</body>
</html>
