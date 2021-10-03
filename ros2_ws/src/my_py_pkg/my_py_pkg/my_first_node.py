#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2021.09.26
author: yasin sahin
written to construct my first ros2 node
"""
import rclpy
from rclpy.node import Node

class MyNode(Node):
    
    def __init__(self):
        super().__init__('py_test')
        self.counter_ = 0
        self.get_logger().info('Hello ROS2!!') # printing Hello ROS2 inside node
        self.create_timer(0.5, self.timer_callback) # calling the timer callback
        
    def timer_callback(self): # creating timer callback to call in specific rate
        self.counter_ += 1  # increasing the counter everytime timer callback is called
        self.get_logger().info('Hello ' + str(self.counter_))
        

def main(args = None):
    rclpy.init(args = args) # initialize ros2 communication
    node = MyNode() # creating node
    rclpy.spin(node) # pausing node to be alive
    rclpy.shutdown() # shutdown ros2 communication

if __name__ == '__main__':
    main()