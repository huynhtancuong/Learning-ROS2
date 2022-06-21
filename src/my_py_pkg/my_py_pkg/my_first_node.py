#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    # The node is not executable, the node is created inside the file
    node = Node("py_test") # the name of the node is not the name of the file
    node.get_logger().info("Hello ROS2")
    rclpy.spin(node) # wait for callback function
    rclpy.shutdown()

if __name__ == "__main__":
    main()
