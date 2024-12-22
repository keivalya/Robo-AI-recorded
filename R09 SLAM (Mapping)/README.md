# SLAM I (Mapping)

## Introduction to Navigation2 Stack in ROS2

Navigation2 (Nav2) is the successor of the ROS Navigation Stack, designed specifically for ROS2. It's a comprehensive framework that enables ground robots to navigate autonomously in complex environments[1][4].

### Where and Why to Use Nav2

Nav2 is used in various applications, including:

- Ground delivery systems
- Hospitals and medical centers
- Hotels for room service
- Offices and warehouses
- Restaurants[7]

The Nav2 stack is particularly useful when you need a robot to:

1. Navigate from a starting point to a goal location safely
2. Avoid obstacles
3. Plan and execute paths in dynamic environments
4. Perform complex navigation tasks using behavior trees[1][4]

### Installing Nav2 Stack

To install Nav2 on ROS2 Humble:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

For simulation purposes, also install the Turtlebot3 packages:

```bash
sudo apt install ros-humble-turtlebot3*
```[5]

### Tools to Use

1. **Gazebo**: For robot simulation
2. **RViz**: For visualization of the robot, map, and navigation data
3. **Nav2 Lifecycle Manager**: To manage the lifecycle of Nav2 nodes
4. **Behavior Trees**: For creating custom navigation behaviors[1][4]

## Introduction to Simultaneous Localization and Mapping (SLAM)

SLAM is a computational problem where a robot constructs or updates a map of an unknown environment while simultaneously keeping track of its location within it[3].

Key components of SLAM:

1. **Sensors**: LiDAR, cameras, or other range-finding devices
2. **Data Extraction**: Algorithms to interpret sensor data
3. **Landmark Identification**: Recognizing distinct features in the environment
4. **Loop Closure**: Recognizing previously visited locations[6]

SLAM is crucial for autonomous navigation in unknown environments, allowing robots to build maps and localize themselves without prior knowledge of their surroundings.

## Hands-on Generating and Saving the Map with SLAM

To generate a map using SLAM with Nav2 and Turtlebot3 in simulation:

1. Launch the Turtlebot3 simulation in Gazebo:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. In a new terminal, launch the SLAM node:

```bash
ros2 launch nav2_bringup slam_toolbox_launch.py
```

3. In another terminal, launch RViz for visualization:

```bash
ros2 launch nav2_bringup rviz_launch.py
```

4. To control the robot and explore the environment, use teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

5. Drive the robot around to explore and build the map. You'll see the map being generated in RViz.

6. Once you're satisfied with the map, save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map_name
```

This will save the map as two files: `map_name.pgm` (the image) and `map_name.yaml` (the metadata)[5].

By following these steps, you'll have successfully generated and saved a map using SLAM, which can be used later for autonomous navigation with the Nav2 stack.

Sources
[1] Nav2 — Nav2 1.0.0 documentation https://docs.nav2.org
[2] ROS2 Navigation 2 with Windows https://ms-iot.github.io/ROSOnWindows/ros2/nav2.html
[3] Simultaneous localization and mapping - Wikipedia https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping
[4] Autonomous robot navigation and Nav2: The first steps. - Foxglove https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps
[5] ROS2 Nav2 Tutorial - The Robotics Back-End https://roboticsbackend.com/ros2-nav2-tutorial/
[6] Understanding SLAM in Robotics and Autonomous Vehicles https://www.flyability.com/blog/simultaneous-localization-and-mapping
[7] The Ultimate Guide to the ROS 2 Navigation Stack – Foxy https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
[8] Build and Install — Nav2 1.0.0 documentation https://docs.nav2.org/development_guides/build_docs/index.html
[9] Nav2 - ROS 2 Navigation Stack - Neobotix Online Documentation https://neobotix-docs.de/ros/ros2/autonomous_navigation.html
[10] Navigation Concepts — Nav2 1.0.0 documentation https://docs.nav2.org/concepts/index.html
