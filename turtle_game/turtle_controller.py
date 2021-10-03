#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.10.01
author: yasin sahin
written to construct a ros2 turtle controller for turtlesim
"""

# importing necessary libraries
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

import numpy as np

# importing to be able to write function inside of a function
from functools import partial


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")

        # declaring linear x velocity gain to be able to interactive change
        self.declare_parameter('lin_vel_x_gain', 2.0)
        # declaring linear y velocity gain to be able to interactive change
        self.declare_parameter('lin_vel_y_gain', 1.0)
        # declaring angular z velocity gain to be able to interactive change
        self.declare_parameter('ang_vel_z_gain', 5.0)
        # declearing control loop frequency to be able to interactive change
        self.declare_parameter('control_loop_frequency', 100)

        self.lin_vel_x_gain_ = self.get_parameter('lin_vel_x_gain').value
        self.lin_vel_y_gain_ = self.get_parameter('lin_vel_y_gain').value
        self.ang_vel_z_gain_ = self.get_parameter('ang_vel_z_gain').value
        self.control_loop_freq_ = self.get_parameter(
            'control_loop_frequency').value

        self.loop_counter_ = 0  # initializing control loop counter
        self.alive_turtle_x_pos_ = []  # initializing list to write alive turtle x positions
        self.alive_turtle_y_pos_ = []  # initializing list to write alive turtle y positions
        self.alive_turtle_theta_ = []  # initializing list to write alive turtle angles
        self.alive_turtle_name_ = []  # initializing list to write alive turtle names
        # initializing float to write master turtle x position
        self.master_turtle_x_pos_ = 0.0
        # initializing float to write master turtle y position
        self.master_turtle_y_pos_ = 0.0
        self.master_turtle_theta_ = 0.0  # initializing float to write master turtle angle

        # creating subscriber to listen alive_turtles topic for alive turtle positions
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, 'alive_turtles', self.callback_alive_turtles, 10)

        # creating subscriber to listen turtle1/pose topic for master turtle positions
        self.master_turtle_subscriber_ = self.create_subscription(
            Pose, 'turtle1/pose', self.callback_master_turtle, 10)

        # creating publisher to publish master turtle velocity commands
        self.master_turtle_publisher_ = self.create_publisher(
            Twist, 'turtle1/cmd_vel', 10)

        # creating timer to control master turtle velocity controls at 100 Hz
        self.timer_ = self.create_timer(
            1/self.control_loop_freq_, self.master_turtle_controller)

        # creating logger to inform turtle controller is started
        self.get_logger().info('Turtle controller is started')

    def callback_alive_turtles(self, msg):
        """

        This function subscribe to alive turtles topic to listen every published alive turtle message

        Parameters
        ----------
        msg : type of message
            message sent to alive turtle topic.

        """
        self.alive_turtle_x_pos_ = []  # clearing list to write alive turtle x positions
        self.alive_turtle_y_pos_ = []  # clearing list to write alive turtle y positions
        self.alive_turtle_theta_ = []  # clearing list to write alive turtle angles
        self.alive_turtle_name_ = []  # clearing list to write alive turtle names

        for turtle in msg.turtles:  # loop to append lists for every alive turtle
            # appending alive turtle name to list
            self.alive_turtle_name_.append(turtle.name)
            # appending alive turtle x position to list
            self.alive_turtle_x_pos_.append(turtle.x)
            # appending alive turtle y position to list
            self.alive_turtle_y_pos_.append(turtle.y)
            # appending alive turtle angle to list
            self.alive_turtle_theta_.append(turtle.theta)

    def callback_master_turtle(self, msg):
        """

        This function subscribe to turtle1/pose topic to listen every published master turtle message

        Parameters
        ----------
        msg : type of message
            message sent to turtle1/pose topic.

        """
        self.master_turtle_x_pos_ = msg.x  # master turtle x position
        self.master_turtle_y_pos_ = msg.y  # master turtle y position
        self.master_turtle_theta_ = msg.theta  # master turtle angle

    def master_turtle_controller(self):
        """

        This function creates master turtle velocity commands to catch nearest alive turtle

        """
        linear_vel = Vector3()  # initializing master turtle linear velocity object
        ang_vel = Vector3()  # initializing master turtle angular velocity object

        try:  # trying to control master turtle

            cos_theta = np.cos(self.master_turtle_theta_)
            sin_theta = np.sin(self.master_turtle_theta_)

            # consturcting rotation matrix to rotate inertial turtle velocities to body turtle velocities
            rotation_matrix = np.array(
                [[cos_theta, -sin_theta], [sin_theta, cos_theta]])

            # creating master turtle -> alive turtles vector components
            error_alive_turtles_x = self.alive_turtle_x_pos_ - np.asarray(
                self.master_turtle_x_pos_)
            error_alive_turtles_y = self.alive_turtle_y_pos_ - np.asarray(
                self.master_turtle_y_pos_)

            # calculating distance between each alive turtle and master turtle
            distance_turtles = np.sqrt(np.square(error_alive_turtles_x) +
                                       np.square(error_alive_turtles_y))

            # finding alive turtle that has minimum distance to master turtle
            list_distance_turtles = list(distance_turtles)
            min_distance_idx = list_distance_turtles.index(
                min(list_distance_turtles))

            # finding angle error to turn master turtle to closest alive turtle
            error_alive_turtles_theta = np.arctan2(
                error_alive_turtles_y[min_distance_idx], error_alive_turtles_x[min_distance_idx]) - self.master_turtle_theta_

            # creating master turtle -> alive turtles vector for closest alive turtle at inertial
            error_inertial = np.array([[error_alive_turtles_y[min_distance_idx]],
                                       [error_alive_turtles_x[min_distance_idx]]])

            # rotating master turtle -> alive turtles vecto to body
            error_body = np.matmul(rotation_matrix, error_inertial)

            # creating linear and angular velocity commands from turtle error
            # a simple P controller is used, and gain is assigned by trial & error
            # assuming that turtle is not going backwards, limiting forward velocity with 0
            linear_vel.x = max(0.0, error_body[1, 0]*self.lin_vel_x_gain_)
            linear_vel.y = error_body[0, 0]*self.lin_vel_y_gain_ 

            # writing master turtle angular velocity into angular velocity message
            ang_vel.z = error_alive_turtles_theta*self.ang_vel_z_gain_

            # is turtle is close enough to closest alive turtle, sending turtle is catched message
            if (distance_turtles[min_distance_idx] < 1) & (self.loop_counter_ % 10 == 0):
                self.call_catch_turtle(
                    self.alive_turtle_name_[min_distance_idx])
            self.loop_counter_ += 1  # increasing control loop counter

        except Exception:  # stopping master turtle when exception occurs
            linear_vel.x = 0.0
            linear_vel.y = 0.0
            ang_vel.z = 0.0

        msg = Twist()  # creating master turtle message object
        msg.linear = linear_vel  # wrtiting master turtle linear velocity into message
        msg.angular = ang_vel  # writing master turtle angular velocity into message
        # publishing master turtle velocity command
        self.master_turtle_publisher_.publish(msg)

    def call_catch_turtle(self, turtle_name):
        """
        This function is used to create client to catch_turtle server
        It tries to reach the server and sends the requested message.

        Parameters
        ----------
        turtle_name : string
            name of turtle to catch.

        """
        client = self.create_client(
            CatchTurtle, 'catch_turtle')  # creating catch_turtle client
        # creating warning when the server cannot be reached
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for service to catch turtle....')

        request = CatchTurtle.Request()  # creating catch_turtle request message object
        request.turtle_name = turtle_name  # writing turtle_name into message

        # sending request to server asynchronosly
        future = client.call_async(request)
        # processing response when the request is done
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, request=request))

    def callback_call_catch_turtle(self, future, request):
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
            # creating logger to inform succesful process
            self.get_logger().info(str(request.turtle_name) +
                                   ' killing process: ' + str(response.success))

        except Exception as e:
            # creating error logger to inform unsucessful process
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)  # initiliazing ros python library
    node = TurtleControllerNode()  # initiliazing node object
    rclpy.spin(node)  # spinning on node object until program is closed
    rclpy.shutdown()  # shutting down ros / killing all nodes when program is closed


if __name__ == "__main__":
    main()
