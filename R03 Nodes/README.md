# ROS2 Python Nodes Tutorial

Welcome to this hands-on tutorial on creating ROS2 nodes using Python! In this guide, we'll explore how to write your first Python node using Object-Oriented Programming (OOP), implement a talker-listener demo, and work with the TurtleSim simulation and teleoperation.

## Prerequisites

Before we begin, make sure you have:

1. ROS2 installed (preferably Foxy or later)
2. Python 3.6 or higher
3. Basic understanding of Python and OOP concepts

## Writing Your First Python Node (OOP Method)

Let's start by creating a simple ROS2 node using the OOP approach. This method provides better modularity and scalability for your ROS2 projects.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Hello ROS2! Count: {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node creates a timer that prints a message every 0.5 seconds. To run it:

1. Save the code in a file named `my_first_node.py`
2. Make the file executable: `chmod +x my_first_node.py`
3. Run the node: `python3 my_first_node.py`

## Talker-Listener Demo

Now, let's implement a simple talker-listener demo using two separate nodes.

### Talker Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Listener Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run these nodes in separate terminals to see them communicate.

## TurtleSim Simulation

TurtleSim is a lightweight simulator for learning ROS2 concepts. Let's create a node to control the turtle:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtle)

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To use this controller:

1. Start TurtleSim: `ros2 run turtlesim turtlesim_node`
2. Run the controller node

## Teleoperation

Teleoperation allows you to control the turtle (or a real robot) using keyboard input. Here's a simple teleop node:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    print("Use WASD keys to move the turtle. Press 'q' to quit.")
    
    while rclpy.ok():
        key = node.get_key()
        if key == 'w':
            node.publish_velocity(0.5, 0.0)
        elif key == 's':
            node.publish_velocity(-0.5, 0.0)
        elif key == 'a':
            node.publish_velocity(0.0, 0.5)
        elif key == 'd':
            node.publish_velocity(0.0, -0.5)
        elif key == 'q':
            break
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run this node after starting TurtleSim to control the turtle with your keyboard.

## Conclusion

This tutorial provides a hands-on introduction to creating ROS2 nodes using Python with OOP, implementing a talker-listener demo, and working with TurtleSim and teleoperation. These examples serve as a foundation for more complex ROS2 applications. Happy coding!

Sources
[1] Write a Minimal ROS2 Python Node - The Robotics Back-End https://roboticsbackend.com/write-minimal-ros2-python-node/
[2] H17 P2 Programming in ROS: Playing with multiple Turtlesim robots https://www.youtube.com/watch?v=K99-EVtzmhI
[3] Create a ROS2 Node with Python and OOP - ROS2 Tutorial 5 https://www.youtube.com/watch?v=wfCuPQ_6VbI
[4] ROS Turtlesim - YouTube https://www.youtube.com/watch?v=xwT7XWflMdc
[5] OOP with ROS in Python - The Robotics Back-End https://roboticsbackend.com/oop-with-ros-in-python/
[6] [F1tenth (F1/10) Lab 1]: Learning ROS with Turtlesim - YouTube https://www.youtube.com/watch?v=wY-b25PUTro
[7] Switch scripts to OOP? : r/learnpython - Reddit https://www.reddit.com/r/learnpython/comments/qxc3dd/switch_scripts_to_oop/
[8] Using turtlesim, ros2, and rqt — ROS 2 Documentation https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
[9] Object-Oriented Programming (OOP) in Python https://realpython.com/python3-object-oriented-programming/
[10] ros-tutorial-robot-control-vision/README_full.md at master - GitHub https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/README_full.md
