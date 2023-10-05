#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import CompressedImage

class DriveDataCollector(Node):
    def __init__(self):
        super().__init__('drive_data_collector')
        timer_period = 0.5

        self.create_subscription(CompressedImage, "/sensing/camera/traffic_light/image_raw/compressed", self.onImage, 1)
        self.inp_img = None
        self.steering = None

    def onImage(self, msg):
        self.inp_img = msg 
    
    def onSteering(self, msg):
        self.steering = msg
        
def main(args=None):
    print('Hi from drive_data_collector')
    rclpy.init(args=args)
    
    node = DriveDataCollector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()