## Understanding Publisher-Subscriber Architecture in ROS2 Robotics

The Publisher-Subscriber (PubSub) architecture is a fundamental communication pattern in ROS2 that enables efficient and modular communication between different components of a robotic system[1][2].

### Key Concepts

- **Publishers**: Nodes that send data on specific topics[2].
- **Subscribers**: Nodes that receive data from topics they're interested in[2].
- **Topics**: Named channels for message exchange[2].

### Benefits in Robotics

1. **Loose Coupling**: Publishers and subscribers operate independently, enhancing modularity[1].
2. **Scalability**: Easy to add new components without modifying existing ones[1].
3. **Flexibility**: Subscribers can filter and choose which topics to listen to[1].

### Common Applications

- **Sensor Data Collection**: Publishing sensor readings for processing or logging[1].
- **Control Systems**: Subscribing to commands for robot control[1].
- **Inter-robot Communication**: Enabling collaborative tasks between multiple robots[1].

## Hands-on: Writing Publisher and Subscriber Nodes

Let's create a simple publisher and subscriber using Python and OOP in ROS2.

### Step 1: Set Up Your ROS2 Workspace

Ensure you have a ROS2 workspace set up. If not, create one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Create a New Package

Create a new Python package for our nodes:

```bash
ros2 pkg create --build-type ament_python py_pubsub
cd py_pubsub/py_pubsub
```

### Step 3: Write the Publisher Node

Create a file named `publisher_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This publisher node:
- Inherits from the `Node` class
- Creates a publisher that publishes `String` messages on the 'topic' topic
- Uses a timer to publish messages every 0.5 seconds[3]

### Step 4: Write the Subscriber Node

Create a file named `subscriber_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This subscriber node:
- Also inherits from the `Node` class
- Creates a subscription to the 'topic' topic
- Defines a callback function that's called whenever a message is received[3]

### Step 5: Update package.xml and setup.py

Ensure your `package.xml` includes necessary dependencies:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
```

Update `setup.py` to include your new nodes:

```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.publisher_node:main',
        'listener = py_pubsub.subscriber_node:main',
    ],
},
```

### Step 6: Build and Run

Build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

Source the setup files:

```bash
source install/setup.bash
```

Run the nodes in separate terminals:

```bash
ros2 run py_pubsub talker
```

```bash
ros2 run py_pubsub listener
```

You should see the publisher sending messages and the subscriber receiving them[3].

This hands-on example demonstrates the basic implementation of the Publisher-Subscriber architecture in ROS2 using Python and OOP principles. It showcases how nodes can communicate asynchronously, enabling modular and scalable robotic systems.

Sources
[1] Understanding the Publish-Subscribe Mechanism in ROS 2 https://www.roboticscontentlab.com/blog/publish-and-subscribe-mechanism-with-ros-2-rclpy/
[2] ROS 2 Communication Basics: Publishers, Subscribers, Topics https://automaticaddison.com/ros-2-communication-basics-publishers-subscribers-topics/
[3] Create a Basic Publisher and Subscriber (Python) | ROS2 Foxy https://automaticaddison.com/create-a-basic-publisher-and-subscriber-python-ros2-foxy/
[4] Create a ROS2 Topic: Publisher and Subscriber Nodes in Python ... https://www.roboticsunveiled.com/ros2-topic-python-and-cpp/
[5] Hands-On ROS2 - Part 1 (Publisher/Subscriber) - YouTube https://www.youtube.com/watch?v=8407qTyBRe0
[6] Writing a simple publisher and subscriber (Python) https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
[7] ROS 2 Generic Publisher and Subscriber — Omniverse IsaacSim https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_generic_publisher_subscriber.html
[8] Understanding ROS 2 nodes with a simple Publisher - Subscriber pair https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Simple-Publisher-Subscriber.html
[9] Writing a simple publisher and subscriber (C++) - ROS Documentation https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
[10] Learn ROS 2 - Intro: Topics - Hadabot https://www.hadabot.com/learn-ros2-intro-topics.html?step=ros2-pub-sub-overview
