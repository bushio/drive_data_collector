#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rclpy
from rclpy.node import Node
import numpy as np
import math
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import PointCloud2
import ros2_numpy 
import argparse

class LidarDataCollector(Node):
    def __init__(self, dirname="lidar_data"):
        super().__init__('lidar_data_collector')

        self.dirname = dirname
        self.save_raw_data = False
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.create_subscription(VelodyneScan, "~/input/velodyne_packets", self.onLidarData, qos_profile=qos_policy)
        self.pub_lidar_to_converter = self.create_publisher(VelodyneScan, "velodyne_packets", 1)
        self.create_subscription(PointCloud2, "~/input/velodyne_point_cloud", self.onPointCloud2, 10)
        
    def onLidarData(self, msg):
        self.lidar = msg
        if self.save_raw_data:
            filename = self.dirname + "/" + repr(self.lidar.header.stamp.sec) + "_" + repr(self.lidar.header.stamp.nanosec)
            packets_array = []
            for p in self.lidar.packets:
                packets_array.append(np.array(p.data))
            np.save(filename, np.array(packets_array))

        self.pub_lidar_to_converter.publish(msg)
    
    def onPointCloud2(self, msg):
        self.get_logger().info("get PointCloud2 {} {}".format(msg.header.stamp.sec, msg.header.stamp.nanosec))

def main(args=None):
    print('Hi from lidar_data_collector')
    rclpy.init(args=None)

    os.makedirs("lidar_data", exist_ok=True)
    node = LidarDataCollector(dirname="lidar_data")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()