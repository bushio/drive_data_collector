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
from nav_msgs.msg import Odometry
from autoware_auto_perception_msgs.msg import DetectedObjects

class DriveDataCollector(Node):
    def __init__(self):
        super().__init__('drive_data_collector')
        timer_period = 0.25
        # PathWithLaneID のsubscriber
        self.create_subscription(PathWithLaneId, 
                                 "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 
                                 self.onPathWithLaneId, 
                                 1)
        # Trajectory のsubscriber
        self.create_subscription(Trajectory, 
                                 "/planning/scenario_planning/trajectory", 
                                 self.onTrajectory, 
                                 1)
        # 制御コマンド のsubscriber
        self.create_subscription(AckermannControlCommand, 
                                 "/control/command/control_cmd", 
                                 self.onControl, 
                                 1)
        # 自己位置、実速度 のsubscriber
        self.create_subscription(Odometry, 
                                 "/awsim/ground_truth/localization/kinematic_state", 
                                 self.onOdometry, 
                                 1)
        self.create_subscription(DetectedObjects,
                                 "/awsim/ground_truth/perception/object_recognition/detection/objects",
                                 self.onDetectObjects,
                                 1)

    
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.path = None
        self.trajectory = None
        self.control = None
        self.position = None
        self.orientation = None
        self.objects = []

        # 保存するディレクトリを作成
        dt_now = datetime.datetime.now()
        self.save_dir = "collected_data/" +  dt_now.strftime('%Y-%m-%d-%H-%M-%S')

        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.save_dir + "/path/", exist_ok=True)
        os.makedirs(self.save_dir + "/trajectory/", exist_ok=True)
        os.makedirs(self.save_dir + "/control/", exist_ok=True)
        os.makedirs(self.save_dir + "/pose/", exist_ok=True)
        os.makedirs(self.save_dir + "/objects/", exist_ok=True)

        self.ids = 0
    
    def onPathWithLaneId(self, msg):
        self.path = msg
        #self.get_logger().info("{}".format(msg))

    def onTrajectory(self, msg):
        self.trajectory = msg
        #self.get_logger().info("{}".format(msg))

    def onControl(self, msg):
        self.control = msg
        #self.get_logger().info("{}".format(msg))
    
    def onOdometry(self, msg):
        self.twist_linear = msg.twist.twist.linear
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        #self.get_logger().info("{}".format(msg))

    def onDetectObjects(self, msg):
        
        for obj in msg.objects:
            data = {}
            data["label"] =  obj.classification[0].label
            data["probability"] = obj.classification[0].probability
            data["existance_probability"] = obj.existence_probability
            data["position"] = np.array([obj.kinematics.pose_with_covariance.pose.position.x,
                            obj.kinematics.pose_with_covariance.pose.position.y,
                            obj.kinematics.pose_with_covariance.pose.position.z])
            data["orientation"] = np.array([obj.kinematics.pose_with_covariance.pose.orientation.x,
                            obj.kinematics.pose_with_covariance.pose.orientation.y,
                            obj.kinematics.pose_with_covariance.pose.orientation.z,
                            obj.kinematics.pose_with_covariance.pose.orientation.w])
            data["twist"] = np.array([obj.kinematics.twist_with_covariance.twist.linear.x])
            self.objects.append(data)
    
            #self.get_logger().info("{}".format(data))

    def timer_callback(self):
        if (self.path is not None) and \
            (self.control is not None) and \
            (self.trajectory is not None) and \
            (self.position is not None) and \
            (self.orientation is not None):
            # 経路の境界線をnumpy形式に変更
            left_bound = self._PointList2Numpy(self.path.left_bound)
            right_bound = self._PointList2Numpy(self.path.right_bound)

            # 生成された経路(Trajectory)をnumpyy形式に変更
            trj_points, trj_vels = self._Trajectory2Numpy(self.trajectory)

            # 制御コマンドをnumpyy形式に変更
            speed = math.ceil(self.control.longitudinal.speed * 3.6)
            accel = self.control.longitudinal.acceleration
            steering_angle = self.control.lateral.steering_tire_angle
            control_cmd = np.array([speed, accel, steering_angle])
            ids_str = '{0:07}'.format(self.ids)

            # Odometory の値をnumpy 形式に変更
            odometry_speed = float(self.twist_linear.x * 3.6)
            position_and_orientation = np.array([self.position.x, 
                                                self.position.y, 
                                                self.position.z,
                                                self.orientation.x,
                                                self.orientation.y,
                                                self.orientation.z,
                                                self.orientation.w])                
            # ファイル名を設定
            control_cmd_file = "{}/{}.npy".format(self.save_dir + "/control/", ids_str)
            trj_file = "{}/{}.npy".format(self.save_dir + "/trajectory/", ids_str)
            path_left_file = "{}/{}_left.npy".format(self.save_dir + "/path/", ids_str)
            path_right_file = "{}/{}_right.npy".format(self.save_dir + "/path/", ids_str)
            pose_file = "{}/{}.npy".format(self.save_dir + "/pose/", ids_str)
            
            # npy 形式で保存
            np.save(control_cmd_file, control_cmd)
            np.save(trj_file, trj_points)
            np.save(path_left_file, left_bound)
            np.save(path_right_file, right_bound)
            np.save(pose_file, position_and_orientation)


            for obj_id, obj in enumerate(self.objects):
                fname = "{}/objects/{}_{}_classify.npy".format(self.save_dir, ids_str, obj_id)
                np.save(fname, np.array([obj["label"],
                                         obj["probability"],
                                         obj["existance_probability"]]))
                
                fname = "{}/objects/{}_{}_pose.npy".format(self.save_dir, ids_str, obj_id)
                np.save(fname, obj["position"])

                fname = "{}/objects/{}_{}_orientation.npy".format(self.save_dir, ids_str, obj_id)
                np.save(fname, obj["orientation"])
                
                fname = "{}/objects/{}_{}_twist.npy".format(self.save_dir, ids_str, obj_id)
                np.save(fname, obj["twist"])

            self.ids += 1
            self.get_logger().info("save {}".format(ids_str))

            # 同じ値を保存しないように初期化
            self.trajectory = None
            self.control = None
            self.trajectory = None
            self.position = None
            self.orientation = None
            self.objects = []

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