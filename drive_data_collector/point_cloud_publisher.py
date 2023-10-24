#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import glob
from sensor_msgs.msg import PointCloud2
import argparse
from .point_cloud_util import point_cloud2_to_array, array_to_point_cloud2


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.declare_parameter('dirname', "LIDAR_DATA_DIR/pcl/")
        dirname = self.get_parameter('dirname').get_parameter_value().string_value
        
        if not os.path.isdir(dirname):
            print("{} is not existed!!".format(dirname))
            exit()

        # Publisher sending raw data to converter
        self.pub_point_cloud = self.create_publisher(PointCloud2, "velodyne_point_cloud", 1)
        
        # Waite time after publishing each data
        self.waite_time = 0.5

        self.publish_data(dirname)

    def publish_data(self, dirname: str, ext: str = ".npy"):
        files = glob.glob(dirname + "/*" + ext)
        for f in files:
            print(f)
            arr = np.load(f, allow_pickle=True)
            #print(arr)
            arr = arr.item()

            d = {"xyz": arr.get("xyz"),
                "intensity": arr.get("intensity")}

            cloud_msg = array_to_point_cloud2(d)
            self.pub_point_cloud.publish(cloud_msg)
            time.sleep(self.waite_time)


def main(args=None):
    print('Hi from point_cloud_publisher')
    
    rclpy.init(args=None)

    node = PointCloudPublisher()
    return 1

if __name__ == '__main__':
    main(args)