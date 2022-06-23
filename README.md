
# What did I do?

I have created a node inside the package name **my_py_pkg**. 
The name of the node is **py_test**, which can be found in the **my_first_node.py**
The name of the executable is **py_node**, which can be found in the **package.xml** file

# How to run this node?

Firstly, we have to build the package using

	$ colcon build

Then you can run the node inside this package using

	$ ros2 run my_py_pkg py_node

# Python Node template
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test")
        self.get_logger().info("Hello ROS2")
        self.get_logger().info("Creating timer")
        self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)
    # The node is not executable, the node is created inside the file
    node = MyNode()
    rclpy.spin(node) # wait for callback function
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

# Python publisher template 
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station")
        self.get_logger().info("Robot News Station has been started")
        self.publishers_ = self.create_publisher(String, "robot_news", 10) # 10 is queue history

        self.create_timer(0.5, self.publish_news)
    
    def publish_news(self):
        msg = String()
        msg.data = "Hello"
        self.publishers_.publish(msg)
        self.get_logger().info("Published news")

def main(args=None):
    rclpy.init(args=args)
    # The node is not executable, the node is created inside the file
    node = RobotNewsStationNode()
    rclpy.spin(node) # wait for callback function
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

# Python subscriber template 
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class SmartphoneNode(Node):

    def __init__(self):
        super().__init__("smartphone")

        self.subcriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)

        self.get_logger().info("Smartphone has been started")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    # The node is not executable, the node is created inside the file
    node = SmartphoneNode()
    rclpy.spin(node)  # wait for callback function
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

# How to add dependence to package
Add dependency to file *package.xml*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_py_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="huynhtancuong.hcm@gmail.com">huynh</maintainer>
  <license>TODO: License declaration</license>

# Add to here 
  <depend>rclpy</depend> 
  <depend>example_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

# How to create executable 
```python
from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huynh',
    maintainer_email='huynhtancuong.hcm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # the name of the executable is py_node
            "py_node = my_py_pkg.my_first_node:main",
            "robot_news_station = my_py_pkg.robot_news_station:main",
            "smartphone = my_py_pkg.smartphone:main"
        ],
    },
)

```