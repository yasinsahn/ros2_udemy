#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.28
author: yasin sahin
written to construct number counter node that counts number published on number_publisher 
and publishes the total counted number

"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__('number_counter')

        self.counter_ = 0  # initializing the number counter

        # creating publisher to publish number_count topic
        self.publisher_ = self.create_publisher(Int64, 'number_count', 10)

        self.subscriber_ = self.create_subscription(
            Int64, 'number', self.callback_number_count, 10)  # creating subscriber to subscribe number topic
        self.server_ = self.create_service(
            SetBool, 'reset_counter', self.callback_reset_counter)  # creating counter resetter server
        # assigning the logger to inform number counter is started
        self.get_logger().info('Number counter is started.')

    def callback_number_count(self, msg):
        """

        This function subscribe to number topic to listen every published number 
        and publishes total number count

        Parameters
        ----------
        msg : type of message
            message sent to number topic.

        """
        # when new number is catched increasing the counter with the number amount
        self.counter_ += msg.data
        msg_count = Int64() # creating number count message object
        msg_count.data = self.counter_ # assigning number count message data
        self.publisher_.publish(msg_count) # publishing number count message

    def callback_reset_counter(self, request, response):
        """

        This function processes the client request and creates the response

        Parameters
        ----------
        request : request type
            request taken from the reset counter client.
        response : response type
            response which will be completed after processing the request.

        Returns
        -------
        response : response type
            response which is completed after processing the request..

        """
        if request.data:  # zeroizing counter if request is True
            self.counter_ = 0
            response.success = True
            response.message = 'Counter is succesfully resetted'
        else:
            response.success = False
            response.message = 'Counter is not ressetted'
        return response


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = NumberCounterNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
