#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.28
author: yasin sahin
written to construct a ros2 service that processes two number addition request
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
# two number addition service message
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        # creating service for sum of two integers
        self.server_ = self.create_service(
            AddTwoInts, 'add_two_ints', self.callback_add_two_ints)
        # creating logger to inform that service is started
        self.get_logger().info('Add two integers server is started')

    def callback_add_two_ints(self, request, response):
        """
        
        This function processes the client request and creates the response

        Parameters
        ----------
        request : request type
            request taken from the two integer addition client.
        response : response type
            response which will be completed after processing the request.

        Returns
        -------
        response : response type
            response which is completed after processing the request..

        """
        response.sum = request.a + request.b  # summing two integers sent by request
        # creating logger to inform successful result
        self.get_logger().info(str(request.a) + ' + ' +
                               str(request.b) + ' is equal to ' + str(response))
        return response


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = AddTwoIntsServerNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
