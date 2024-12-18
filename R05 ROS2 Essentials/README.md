# ROS2 Essentials

## Fundamentals of ROS2

ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Concepts

1. **Nodes**: Executable processes that perform computation.
2. **Topics**: Named buses over which nodes exchange messages.
3. **Services**: Request/response interactions between nodes.
4. **Actions**: For long-running tasks with feedback.
5. **Parameters**: Configuration values for nodes.

## Topics

Topics are named buses over which nodes exchange messages. Publishers send messages to a topic, while subscribers receive messages from a topic.

Here's an example of a simple publisher node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services are synchronous call-and-response interactions between nodes. Here's an example of a simple service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes simultaneously. Here's an example of a simple launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtle1/pose'),
                ('/output/cmd_vel', '/turtle2/cmd_vel'),
            ]
        )
    ])
```

## Workspace and Packages

A ROS2 workspace is a directory containing ROS2 packages. Here's how to create a workspace and a package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_package
cd ~/ros2_ws
colcon build
```

## Understanding Transforms (TFs)

TF2 is the transform library in ROS2, allowing you to keep track of multiple coordinate frames over time. Here's an example of broadcasting a static transform:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'static_frame'
        static_transformStamped.transform.translation.x = 1.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(static_transformStamped)

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This code demonstrates the core concepts of ROS2, including nodes, topics, services, launch files, workspaces, packages, and transforms. These elements form the foundation for building complex robotic systems using ROS2[1][2][3][4][5][6][7][8][9][10].

## Sources

- [1] ROS2 Python Launch File Example - How to Start All Your Nodes at ... https://roboticsbackend.com/ros2-launch-file-example/
- [2] Tutorial: ROS2 launch files - All you need to know - Robotics Casual https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/
- [3] The Transform System (tf2) - Articulated Robotics https://articulatedrobotics.xyz/tutorials/ready-for-ros/tf/
- [4] Tf2 — ROS 2 Documentation: Humble documentation https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html
- [5] ROS2 Basics Exercise - ROS Industrial Training - Read the Docs https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS2-Basics.html
- [6] Integrating launch files into ROS 2 packages https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html
- [7] Introducing tf2 — ROS 2 Documentation: Foxy documentation https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- [8] Launch — ROS 2 Documentation: Foxy documentation https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html
- [9] How to Use ROS 2 Launch Files - Foxglove https://foxglove.dev/blog/how-to-use-ros2-launch-files
- [10] Launch Files - ROS Industrial Training - Read the Docs https://industrial-training-master.readthedocs.io/en/foxy/_source/session2/ros2/2-Launch-Files.html
