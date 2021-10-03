#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.28
author: yasin sahin
written to construct a ros2 client that sends two number addition request
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
# two number addition service message
from example_interfaces.srv import AddTwoInts
# importing to be able to write function inside of a function
from functools import partial


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        # calling the two integer addition server
        self.call_add_two_ints_server(6, 7)

    def call_add_two_ints_server(self, a, b):
        """
        This function is used to create client to two integer addition server
        It tries to reach the server and sends the requested message.

        Parameters
        ----------
        a : int
            first integer to use in addition process.
        b : int
            second integer two use in addition process.

        """
        client = self.create_client(
            AddTwoInts, 'add_two_ints')  # creating client for two integer addition
        # creating warning when server is not reached
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for service to add two Ints....')

        request = AddTwoInts.Request()  # creating request message
        request.a = a  # implementing first integer to request message
        request.b = b  # implementing second integer to request message

        # sending request to server asynchronosly
        future = client.call_async(request)
        # processing response when the request is done
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        """
        This function processes the response from the service

        Parameters
        ----------
        future : response type
            response taken from server for the request.
        a : int
            first integer used in addition process.
        b : int
            second integer used in addition process.

        """
        try:
            response = future.result()  # taking response from the service
            # creating logger to inform succesful process
            self.get_logger().info(str(a) + ' + ' + str(b) + ' = ' + str(response.sum))

        except Exception as e:
            # creating error logger to inform unsucessful process
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = AddTwoIntsClient() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
