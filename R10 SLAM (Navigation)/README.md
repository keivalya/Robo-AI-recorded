# SLAM II (Navigation)

This guide will walk you through navigating a TurtleBot3 using a previously generated map, waypoint following, dynamic obstacle avoidance, and understanding global and local planning methods.

---

### Hands-on: Navigate Using a Generated Map

Once you've generated and saved a map using SLAM, you can use it for autonomous navigation.

#### Steps to Navigate:
1. **Launch the TurtleBot3 Simulation**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch the Navigation2 Stack**:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map:=/path/to/your/map.yaml
   ```

3. **Send a Navigation Goal**:
   Open RViz:
   ```bash
   ros2 launch nav2_bringup rviz_launch.py
   ```
   In RViz, use the "2D Nav Goal" tool to click on the map and set a goal for the robot. The robot will plan a path and navigate autonomously to the goal.

---

### Waypoint Following for TurtleBot3

Waypoint following allows the robot to navigate through multiple predefined points sequentially.

#### Steps to Implement Waypoint Following:
1. **Create a Waypoint File**:
   Define waypoints in a YAML file (e.g., `waypoints.yaml`):
   ```yaml
   waypoints:
     - {x: 1.0, y: 1.0, theta: 0.0}
     - {x: 2.0, y: 1.5, theta: 0.0}
     - {x: 3.0, y: 2.0, theta: 0.0}
   ```

2. **Run Waypoint Navigator**:
   Use the `waypoint_follower` node from Navigation2 or write your own script to publish goals sequentially to `/navigate_to_pose`.

Example Python script for waypoint navigation:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.publisher = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        self.timer = self.create_timer(5.0, self.publish_waypoint)
        self.waypoints = [
            {'x': 1.0, 'y': 1.0, 'theta': 0.0},
            {'x': 2.0, 'y': 1.5, 'theta': 0.0},
            {'x': 3.0, 'y': 2.0, 'theta': 0.0}
        ]
        self.current_index = 0

    def publish_waypoint(self):
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint['x']
            pose.pose.position.y = waypoint['y']
            pose.pose.orientation.z = waypoint['theta']
            self.publisher.publish(pose)
            self.get_logger().info(f"Publishing waypoint {self.current_index + 1}")
            self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Dynamic Obstacle Avoidance

Dynamic obstacle avoidance ensures that the robot can react to moving obstacles in real-time while following its planned path.

#### How It Works:
- **Sensors**: LiDAR or depth cameras detect obstacles in real-time.
- **Local Planner**: Adjusts the robot's trajectory dynamically to avoid obstacles while staying on course.

#### Example Implementation:
The Dynamic Window Approach (DWA) is often used as the local planner in ROS2 Navigation2.

To enable dynamic obstacle avoidance in Nav2:
1. Ensure your `local_costmap` is configured correctly in your Nav2 parameters file (`nav2_params.yaml`):
   ```yaml
   local_costmap:
     plugin: "voxel_layer"
     rolling_window: true
     width: 5
     height: 5
     resolution: 0.05
     inflation_radius: 0.5
     cost_scaling_factor: 10.0
   ```

2. Adjust parameters for obstacle detection and inflation radius to suit your environment.

---

### Understanding Global and Local Planning Methods

#### Global Planning:
- **Purpose**: Plans an optimal path from start to goal based on the entire map.
- **Algorithms Used**:
  - A* (default in ROS2 Nav2)
  - Dijkstra's algorithm

#### Local Planning:
- **Purpose**: Reacts to immediate obstacles and adjusts the trajectory while following the global path.
- **Algorithms Used**:
  - Dynamic Window Approach (DWA)
  - Timed Elastic Band (TEB)

#### Workflow of Global and Local Planners:
1. The **global planner** computes an initial path from start to goal using the global costmap.
2. The **local planner** continuously adjusts this path based on sensor data from the local costmap.
3. If dynamic obstacles invalidate the global path, replanning is triggered.

---

### Summary

By combining SLAM-generated maps with Navigation2's capabilities, you can autonomously navigate TurtleBot3 through complex environments with real-time obstacle avoidance and waypoint following functionality.

Key takeaways include:
- Using Nav2 for autonomous navigation with global and local planners.
- Configuring costmaps for dynamic obstacle avoidance.
- Implementing waypoint navigation for sequential goals.

These tools make TurtleBot3 an excellent platform for learning advanced robotics concepts like SLAM and autonomous navigation!

Sources
[1] Obstacle Avoider - Stretch Documentation https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/obstacle_avoider/
[2] TurtleBot3 Obstacle Avoidance with Sim2Real Transfer [Tutorial] https://www.youtube.com/watch?v=CQ5qTQAW3HM
[3] Hybrid Planning - MoveIt 2 - PickNik Robotics https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html
[4] [PDF] Navigation2 Overview https://roscon.ros.org/2019/talks/roscon2019_navigation2_overview_final.pdf
[5] ADVRHumanoids/ProximityBasedDynamicCollisionAvoidance https://github.com/ADVRHumanoids/ProximityBasedDynamicCollisionAvoidance
[6] ROS 2 | TurtleBot3 Obstacle Avoidance [Tutorial] - YouTube https://www.youtube.com/watch?v=OmBesHAbSDU
[7] Navigation | Husarion https://husarion.com/tutorials/ros2-tutorials/9-navigation/
[8] Navigation in ROS2 - Robot & Chisel https://www.robotandchisel.com/2020/09/01/navigation2/
[9] Setting up a robot simulation (Advanced) — ROS 2 Documentation https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html
[10] What global and local path planning algorithms do ros use? https://robotics.stackexchange.com/questions/93133/what-global-and-local-path-planning-algorithms-do-ros-use
