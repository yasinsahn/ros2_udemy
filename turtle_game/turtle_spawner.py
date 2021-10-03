#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.10.01
author: yasin sahin
written to construct a turtle spawner for turtlesim

"""

# importing necessary libraries
import rclpy
from rclpy.node import Node

import math
from random import random
# importing to be able to write function inside of a function
from functools import partial

from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.msg import Turtle, TurtleArray


class TurtleSpawnerClientNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_client")

        self.turtles_ = []  # initializing list to append every alive turtle

        # declaring turtle spawn second to be able to interactive change
        self.declare_parameter('turtle_spawn_second', 0.8)

        self.turtle_spawn_second_ = self.get_parameter('turtle_spawn_second').value


        # creating service to catch turtle when request is sent
        self.catch_turtle_server_ = self.create_service(
            CatchTurtle, 'catch_turtle', self.callback_catch_turtle)

        # creating timer to spawn turtle
        self.turtle_spawn_timer_ = self.create_timer(
            self.turtle_spawn_second_, self.call_spawn_turtle)

        # creating publisher to publish alive turtles
        self.turtle_publisher_ = self.create_publisher(
            TurtleArray, 'alive_turtles', 10)

        # creating logger to inform turtle spawner is started
        self.get_logger().info('Turtle spawner is started')

    def call_spawn_turtle(self):
        """
        This function is used to create client to turtle spawn server
        It tries to reach the server and sends the requested message.

        """
        client = self.create_client(
            Spawn, 'spawn')  # creating client for spawn server

        # creating warning when the server cannot be reached
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for service to spawn turtle....')

        request = Spawn.Request()  # creating request object
        # randomly generating turtle spawn x location between [0,11)
        request.x = random()*11
        # randomly generating turtle spawn y location between [0,11)
        request.y = random()*11
        # randomly generating turtle spawn angle between [0,2*pi)
        request.theta = random()*2*math.pi

        # sending request to server asynchronosly
        future = client.call_async(request)
        # processing response when the request is done
        future.add_done_callback(
            partial(self.callback_spawn_turtle, request=request))

    def callback_spawn_turtle(self, future, request):
        """
        This function processes the response from the service

        Parameters
        ----------
        future : response type
            response taken from server for the request.
        request : request type
            request sent from service.

        """

        try:
            response = future.result()  # taking response from the service
            # creating logger to inform turtle is spawned.
            self.get_logger().info('Turtle: ' + response.name + ' is spawned!')
            turtle = Turtle()  # creating turtle object
            turtle.name = response.name  # writing turtle name
            turtle.x = request.x  # writing turtle x-position
            turtle.y = request.y  # writing turtle y-position
            turtle.theta = request.theta  # writing turtle angle
            # appending spawned turtle to alive turtles
            self.turtles_.append(turtle)
            self.publish_alive_turtles()  # publishing alive turtles

        except Exception as e:
            # creating error logger to inform unsucessful process
            self.get_logger().error('Service call failed %r' % (e,))

    def call_kill_turtle(self, turtle_name):
        """
        This function is used to create client to turtle kill server
        It tries to reach the server and sends the requested message.

        """

        # creating client for kill server
        client = self.create_client(Kill, 'kill')

        # creating warning when the server cannot be reached
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for service to kill turtle....')

        request = Kill.Request()  # creating request object
        request.name = turtle_name  # writing turtle name to request

        # sending request to server asynchronosly
        future = client.call_async(request)

        # processing response when the request is done
        future.add_done_callback(
            partial(self.callback_kill_turtle, request=request))

    def callback_kill_turtle(self, future, request):
        """
        This function processes the response from the service

        Parameters
        ----------
        future : response type
            response taken from server for the request.
        request : request type
            request sent from service.

        """
        try:

            future.result()  # taking response from the service
            for turtle in self.turtles_:
                if turtle.name == request.name:
                    # removing killed turtle from alive turtle list
                    self.turtles_.remove(turtle)
            self.publish_alive_turtles()  # publishing alive turtles
            # creating logger to inform turtle is killed
            self.get_logger().info('Turtle: ' + request.name + ' is killed!')

        except Exception as e:
            # creating error logger to inform unsucessful process
            self.get_logger().error('Service call failed %r' % (e,))

    def callback_catch_turtle(self, request, response):
        """
        This function is used to create service to catch_turtle server
        Creates killing process for given turtle name

        Parameters
        ----------
        request : request type
            request taken from client.
        response : response type
            response response to be filled.

        Returns
        -------
        response : response type
            filled service response.

        """
        try: # Creating kill turtle request for given turtle name
            self.call_kill_turtle(request.turtle_name)
            response.success = True
        except Exception:
            response.success = False
        return response

    def publish_alive_turtles(self):
        """
        This function publishes alive turtle list when new turtle spawned
        or an alive turtle killed

        """
        msg = TurtleArray() # creating message to publish
        msg.turtles = self.turtles_ # adding alive turtle list to message
        self.turtle_publisher_.publish(msg) # publishing message


def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = TurtleSpawnerClientNode() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed

if __name__ == "__main__":
    main()
