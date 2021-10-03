#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.28
author: yasin sahin
written to construct ros2 robot news station publisher to robot_news topic
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
# now this package is depend on example_interfaces also, it has to be written to 'package.xml'
from example_interfaces.msg import String  # importing message type of interface


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__('robot_news_station')

        # declaring robot name to be able to interactive change
        self.declare_parameter('robot_name', 'C3PO')

        # assigning robot name for make it more personal
        self.robot_name_ = self.get_parameter('robot_name').value

        # inputs are message_type, topic name, qos profile {buffer size}
        self.publisher_ = self.create_publisher(
            String, 'robot_news', 10)  # creating the publisher variable on robot_news topic
        
        self.timer_ = self.create_timer(
            0.5, self.publish_news)  # publishing data at 2 Hz
        
        # assigning the logger to inform robot news statin is started
        self.get_logger().info('Robot News Station has been started')

    def publish_news(self):
        """
        This function creates and publishes message used for news publishing

        """
        
        msg = String()  # creating the message variable
        # writing the data inside message
        msg.data = 'Hello' + 'This is ' + \
            str(self.robot_name_) + ' from the robot news station'
        self.publisher_.publish(msg)  # publishing message


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = RobotNewsStationNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
