#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.28
author: yasin sahin
written to construct number publisher node that publishes a number
"""
# importing necessary libraries
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__('number_publisher')

        # declaring published number to be able to interactive change
        self.declare_parameter('number_to_publish', 2)
        # declearing number publishing frequency to be able to interactive change
        self.declare_parameter('publish_frequency', 1.0)

        # number that will be published
        self.number_ = self.get_parameter('number_to_publish').value
        # creating publisher to publish the number on number topic
        self.publisher_ = self.create_publisher(Int64, 'number', 10)

        # number publishing frequency
        self.publish_frequency_ = self.get_parameter('publish_frequency').value

        # publishing the number at publishing frequency
        self.create_timer(1.0 / self.publish_frequency_, self.publish_number)

        # assigning the logger to inform number publisher is started
        self.get_logger().info('Number publisher is started')

    def publish_number(self):
        """
        This function creates and publishes message used for number publishing

        """
        msg = Int64()  # creating message
        msg.data = self.number_  # assigning message data
        self.publisher_.publish(msg)  # publishing message data


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = NumberPublisherNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed

if __name__ == "__main__":
    main()
