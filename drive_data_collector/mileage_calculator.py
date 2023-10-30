#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import math
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import sys

class MileageCalc(Node):
    def __init__(self):
        super().__init__('mileage_calculator')
        
        ## Vehicle odometry subscriber ##
        #self.create_subscription(Odometry, "~/input/odometry", self.onOdometry, 10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.onOdometry, 10)
        # Publisher sending raw data to converter
        self.pub_mileage = self.create_publisher(Float32, "drive_data_collector/mileage", 1)
        self.mileage  = 0.0
        self.previous_pose = None

        self.minimum_dist = 0.5

    def onOdometry(self, msg: Odometry):
        self.current_odometry = msg
        self.ego_pose = self.current_odometry.pose.pose

        if self.previous_pose is None:
            self.previous_pose = self.ego_pose
        else:
            diff = math.sqrt((self.ego_pose.position.x - self.previous_pose.position.x) ** 2 + (self.ego_pose.position.y - self.previous_pose.position.y) ** 2)
            if diff > self.minimum_dist:
                self.mileage += diff
                self.previous_pose = self.ego_pose
                pub_msg = Float32()
                print(self.mileage)
                pub_msg.data = self.mileage
                self.pub_mileage.publish(pub_msg)
            


def main(args=None):
    print('Hi from mileage_calculator')
    rclpy.init(args=None)

    node = MileageCalc()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()