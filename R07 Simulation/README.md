# Simulation

## Running Gazebo

To run Gazebo, open a terminal and enter the following command:

```bash
gazebo
```

This will launch Gazebo with a default world. To load a specific world file, use:

```bash
gazebo worlds/pioneer2dx.world
```

## How Gazebo Works

Gazebo consists of two main components:

1. **gzserver**: The physics engine and sensor simulation core.
2. **gzclient**: The graphical user interface.

Gazebo uses SDF (Simulation Description Format) to describe the world and models within it. It can convert URDF (Unified Robot Description Format) to SDF automatically[1][2].

## Adding Inertia and Collision Tags in URDF

To add inertia and collision tags to your URDF:

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <collision>
    <geometry>
      <box size="0.5 0.5 0.5"/>
    </geometry>
  </collision>
  <visual>
    <geometry>
      <box size="0.5 0.5 0.5"/>
    </geometry>
  </visual>
</link>
```

## Spawning the Robot

To spawn a robot in Gazebo using ROS 2:

```bash
ros2 run gazebo_ros spawn_entity -topic robot_description -entity my_robot
```

This command reads the URDF from the `robot_description` topic and spawns it in Gazebo[2].

## Fixing Inertia Values

Ensure inertia values are physically realistic. For a solid cuboid:

```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.0833" ixy="0.0" ixz="0.0" iyy="0.0833" iyz="0.0" izz="0.0833"/>
</inertial>
```

## Creating a World in Gazebo

Create a new SDF file (e.g., `my_world.sdf`) with the following content:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

## Launching Robot in the World

Create a launch file (e.g., `launch_robot.py`) to launch Gazebo with your world and spawn the robot:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'my_world.sdf'],
            output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
            output='screen'
        ),
    ])
```

Run the launch file with:

```bash
ros2 launch launch_robot.py
```

This will start Gazebo with your custom world and spawn your robot in it[3][4][5].

Sources
[1] Tutorial: Gazebo Simulation - Fetch & Freight Manual https://docs.fetchrobotics.com/gazebo.html
[2] Simulating Robots with Gazebo and ROS - YouTube https://www.youtube.com/watch?v=laWn7_cj434
[3] Tutorial : Quick Start - Gazebo https://classic.gazebosim.org/tutorials?tut=quick_start
[4] Setting up a robot simulation (Gazebo) — ROS 2 Documentation https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
[5] Getting Started with Gazebo? — Gazebo ionic documentation https://gazebosim.org/docs/latest/getstarted/
