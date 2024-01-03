#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rclpy
import numpy as np
from rclpy.node import Node
import math
import datetime
import os
from autoware_auto_planning_msgs.msg import Trajectory, PathWithLaneId
from autoware_auto_control_msgs.msg import AckermannControlCommand

class DriveDataCollector(Node):
    def __init__(self):
        super().__init__('drive_data_collector')
        timer_period = 0.1

        self.create_subscription(PathWithLaneId, 
                                 "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 
                                 self.onPathWithLaneId, 
                                 1)
        self.create_subscription(Trajectory, 
                                 "/planning/scenario_planning/trajectory", 
                                 self.onTrajectory, 
                                 1)

        self.create_subscription(AckermannControlCommand, 
                                 "/control/command/control_cmd", 
                                 self.onControl, 
                                 1)
    
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.path = None
        self.trajectory = None
        self.control = None

        # 保存するディレクトリを作成
        dt_now = datetime.datetime.now()
        self.save_dir = "datasets/" +  dt_now.strftime('%Y-%m-%d-%H-%M-%S')

        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.save_dir + "/path/", exist_ok=True)
        os.makedirs(self.save_dir + "/trajectory/", exist_ok=True)
        os.makedirs(self.save_dir + "/control/", exist_ok=True)

        self.ids = 0
    
    def onPathWithLaneId(self, msg):
        self.path = msg
        #self.get_logger().info("{}".format(self.path))

    def onTrajectory(self, msg):
        self.trajectory = msg
        #self.get_logger().info("{}".format(self.trajectory))

    def onControl(self, msg):
        self.control = msg
        #self.get_logger().info("{}".format(self.control))
    
    def timer_callback(self):
        if (self.path is not None) and (self.control is not None) and (self.trajectory is not None):
            left_bound = self._PointList2Numpy(self.path.left_bound)
            right_bound = self._PointList2Numpy(self.path.right_bound)
            trj_points, trj_vels = self._Trajectory2Numpy(self.trajectory)
            speed = math.ceil(self.control.longitudinal.speed * 3.6)
            accel = self.control.longitudinal.acceleration
            steering_angle = self.control.lateral.steering_tire_angle
            control_cmd = np.array([speed, accel, steering_angle])
            ids_str = '{0:07}'.format(self.ids)

            control_cmd_file = "{}/{}.npy".format(self.save_dir + "/control/", ids_str)
            trj_file = "{}/{}.npy".format(self.save_dir + "/trajectory/", ids_str)
            path_left_file = "{}/{}_left.npy".format(self.save_dir + "/path/", ids_str)
            path_right_file = "{}/{}_right.npy".format(self.save_dir + "/path/", ids_str)
            np.save(control_cmd_file, control_cmd)
            np.save(trj_file, trj_points)
            np.save(path_left_file, left_bound)
            np.save(path_right_file, right_bound)


            self.ids += 1
            # self.get_logger().info("{}".format(self.control))

    def _PointList2Numpy(self, points):
        k = []
        for i in range(len(points)):
            k.append([points[i].x, points[i].y])
        return np.array(k)
    
    def _Trajectory2Numpy(self, trajectory):
        points_vel_list = []
        points_pose_list = []
        for p in trajectory.points:
            points_vel_list.append(p.longitudinal_velocity_mps)
            points_pose_list.append([p.pose.position.x, p.pose.position.y])
        return np.array(points_pose_list), np.array(points_vel_list)

def main(args=None):
    print('Hi from drive_data_collector')

    rclpy.init(args=args)
    
    node = DriveDataCollector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()