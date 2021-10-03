#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.28
author: yasin sahin
written to construct ros2 subscriber to robot_news topic
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
# now this package is depend on example_interfaces also, it has to be written to 'package.xml'
from example_interfaces.msg import String  # importing message type of interface


class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        # inputs are message_type, topic name, callback function, qos profile {buffer size}
        self.subscriber_ = self.create_subscription(
            String, 'robot_news', self.callback_robot_news, 10)  # creating subscriber on robot_news topic
        self.get_logger().info('Smartphone has been started.')  # assigning the logger

    def callback_robot_news(self, msg):
        """

        This function subscribe to robot_news topic to listen every published message 

        Parameters
        ----------
        msg : type of message
            message sent to robot_news topic.

        """
        self.get_logger().info(msg.data)  # creating logger when message is captured


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = SmartphoneNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
