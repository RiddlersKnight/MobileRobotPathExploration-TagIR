# Tag IR Simulator


Dinithi Silva

Dartmouth College - Robotics Reality Lab

This project is built on the demo Unity ROS project as it provides all of the necessary libraries and components to use ROS with Unity. Please refer to README-Original for more information about that base project.

# Setup

* The ROS-TCP-Endpoint should be already built, so just `source MobileRobotPathExploration-TagIR/ros2_docker/colcon_ws/install/setup.bash` . If that does not work, you may have to build the colcon workspace. See the README-Original.md for more details
* Open the Unity project located in the folder `MobileRobotPathExploration-TagIR/MobileRobotPathExploration-TagIR`. You should use Unity Editor version 2020.3.11f1.
* To try the Wall Follower, open any scene in the folder `Assets/Robot_Wall_Follower/Scenes`. Maze size is indicated in the name of the Scene.
* To try the IR Tag exploration, open any scene in the folder `Assets/Robot_Localization_Light_Tag/Scenes`. Maze size is indicated in the name of the Scene.
