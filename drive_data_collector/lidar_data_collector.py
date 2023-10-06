#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import math
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import PointCloud2
import argparse
sys.path.append("ros2_numpy")
import ros2_numpy as rnp

# Save directory
DIR_NAME = "LIDAR_DATA_DIR"

# Save flag for lidar raw data
SAVE_RAW_DATA = False

# Save flag for lidar point cloud
SAVE_PCL2_DATA = True

class LidarDataCollector(Node):
    def __init__(self, dirname="LIDAR_DATA"):
        super().__init__('lidar_data_collector')
        
        # Directory for raw data
        self.dirname_raw_data = dirname + "/raw"
        os.makedirs(self.dirname_raw_data, exist_ok=True)
        self.save_raw_data = SAVE_RAW_DATA

        # Directory for point cloud
        self.dirname_pcl = dirname + "/pcl"
        os.makedirs(self.dirname_pcl, exist_ok=True)
        self.save_pcl2_data = SAVE_PCL2_DATA

        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        # Subscriber for raw data
        self.create_subscription(VelodyneScan, "~/input/velodyne_packets", self.onLidarRawData, qos_profile=qos_policy)

        # Subscriber for point cloud
        self.create_subscription(PointCloud2, "~/input/velodyne_point_cloud", self.onLidarPointCloud2, 10)

        # Publisher sending raw data to converter
        self.pub_raw_data = self.create_publisher(VelodyneScan, "velodyne_packets", 1)
    
    def onLidarRawData(self, msg: VelodyneScan):

        if self.save_raw_data:
            filename = self.dirname_raw_data + "/" + repr(msg.header.stamp.sec) + "_" + repr(msg.header.stamp.nanosec)
            packets_array = []
            for p in self.lidar.packets:
                packets_array.append(np.array(p.data))
            np.save(filename, np.array(packets_array))

        self.pub_raw_data.publish(msg)
    
    def onLidarPointCloud2(self, msg: PointCloud2):
        self.get_logger().info("get PointCloud2 {} {}".format(msg.header.stamp.sec, msg.header.stamp.nanosec))
        arr = rnp.point_cloud2.point_cloud2_to_array(msg)

        if self.save_pcl2_data:
            filename = self.dirname_pcl + "/" + repr(msg.header.stamp.sec) + "_" + repr(msg.header.stamp.nanosec)
            np.save(filename, arr)

def main(args=None):
    print('Hi from lidar_data_collector')
    rclpy.init(args=None)

    os.makedirs(DIR_NAME, exist_ok=True)
    node = LidarDataCollector(dirname=DIR_NAME)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()