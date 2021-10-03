#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.29
author: yasin sahin
written to construct led panel node to activate and publishes led states
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
# importing LedStates message from my_robot_interfaces
from my_robot_interfaces.msg import LedStates


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel_node")

        self.led_state1_ = False
        self.led_state2_ = False
        self.led_state3_ = False  # initializing the led states

        self.server_ = self.create_service(
            SetLed, 'set_led', self.callback_set_led)  # creating service server for setting leds

        # creating publisher to publish led_states topic
        self.publisher_ = self.create_publisher(LedStates, 'led_states', 10)

        # creating timer to publish led sates every 4 seconds
        self.timer_ = self.create_timer(4.0, self.publish_led_states)

        # creating logger to indicate battery is started
        self.get_logger().info('Led Panel is started')

    def callback_set_led(self, request, response):
        """

        This function processes the client request and creates the response

        Parameters
        ----------
        request : request type
            request taken from the battery client.
        response : response type
            response which will be completed after processing the request.

        Returns
        -------
        response : response type
            response which is completed after processing the request..

        """
        response.success = True  # indicating success of the request
        self.led_state1_ = request.led_state1  # assigning led state to publish
        self.led_state2_ = request.led_state2  # assigning led state to publish
        self.led_state3_ = request.led_state3  # assigning led state to publish
        
        # creating logger to inform low charge status
        self.get_logger().info('Low charge status: ' + str(request.led_state3) +
                               ' Led states apply status: ' + str(response.success))
        
        return response

    def publish_led_states(self):
        """
        
        This function creates and publishes message used for led state publishing

        """
        msg = LedStates() # creating message object
        msg.led_state1 = self.led_state1_ # assigning led state on the message
        msg.led_state2 = self.led_state2_ # assigning led state on the message
        msg.led_state3 = self.led_state3_ # assigning led state on the message
        self.publisher_.publish(msg) # publishing the message


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = LedPanelNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
