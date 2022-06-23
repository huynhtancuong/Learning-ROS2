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
