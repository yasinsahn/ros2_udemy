#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.29
author: yasin sahin
written to construct hardware status publisher for ros2 
using custom-created hardware status custom message
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
# importing HardwareStatus message from my_robot_interfaces
from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")

        # creating publisher to publish hardware_status topic
        self.hw_status_publisher_ = self.create_publisher(
            HardwareStatus, 'hardware_status', 10)
        # creating timer to publish hardware status every 1 second
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        # creating logger to inform hardware status publisher is started
        self.get_logger().info('Hardware status publisher is started!')

    def publish_hw_status(self):
        """
        This function creates and publishes message used for hardware status publishing

        """
        msg = HardwareStatus() # creating message object
        msg.temperature = 45 # assigning hardware temperature
        msg.are_motors_ready = True # assigning motor status
        msg.debug_message = 'Nothing Special' # assigning debug message
        self.hw_status_publisher_.publish(msg) # publishing the message


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = HardwareStatusPublisherNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
