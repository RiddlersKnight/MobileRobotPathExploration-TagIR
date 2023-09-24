# Tag IR Simulator


Dinithi Silva


Dartmouth College - Robotics Reality Lab

This project is built on the demo Unity ROS project as it provides all of the necessary libraries and components to use ROS with Unity. Please refer to README-Original for more information about that base project.

# Setup

* The ROS-TCP-Endpoint should be already built, so just `source MobileRobotPathExploration-TagIR/ros2_docker/colcon_ws/install/setup.bash` . If that does not work, you may have to build the colcon workspace. See the README-Original.md for more details
* Open the Unity project located in the folder `MobileRobotPathExploration-TagIR/MobileRobotPathExploration-TagIR`. You should use Unity Editor version 2020.3.11f1.
* To try the Wall Follower, open any scene in the folder `Assets/Robot_Wall_Follower/Scenes`. Maze size is indicated in the name of the Scene.
* To try the IR Tag exploration, open any scene in the folder `Assets/Robot_Localization_Light_Tag/Scenes`. Maze size is indicated in the name of the Scene.

# Explanation of color scheme and visual elements

![Explanation of the color scheme](/readmes/TagIR_images/TagIRSimExplanationCircles.png "Explanation of the color scheme")

The nodes will update over time as the robots explore. Each color means the following to the robot that changed it:

* White - Unexplored nodes
* Black - Dead end
* Blue - Visited by the current robot
* Red - Visited by a different robot
* Orange - Current destination

You will also the see the following visual elements:

* Colored cubes near walls - The lidar hit locations. Red is closer, yellow is further away
* White line between a robot and a tag - Line of sight beam
* Colored lines radiating off of the robots - Effective range of IR light

# Extra images

![Near the start of a run](/readmes/TagIR_images/TagIRSimStart.gif "Near the start of a run")

Near the start of a run

![Near the end of a run](/readmes/TagIR_images/TagIRSimEnd.gif "Near the end of a run")

Near the end of a run