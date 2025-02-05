#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionPlanRequest, RobotTrajectory, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState


class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_client')
        self.mode = 'start'
        self.arm = None 
        self._action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._planning_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        try:
            arm = XArmAPI('192.168.1.192', baud_checkset=False)
            self._arm = arm
            self._arm.connect()  # XArm 연결 시도
            self.get_logger().info("Connected to XArm")
        
        except Exception as e:
            self.get_logger().error(f"Error connecting to the arm: {str(e)}")
            return            
        
    def create_pose_constraint(self, pose):
        constraints = Constraints()

        # 🔹 PositionConstraint 생성
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"  # 기준 좌표계
        position_constraint.link_name = "link6"  # 제약을 적용할 링크

        # BoundingVolume 생성
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE  # 구형 제약 설정
        primitive.dimensions = [0.01]  # 허용 반경 (1cm)

        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(pose)  # Pose 객체 그대로 추가

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # 🔹 OrientationConstraint 생성
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "link6"
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        # 🔹 Constraints에 추가
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def compute_motion_plan(self, target_pose):
        if not self._planning_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Planning service not available")
            return None

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = "lite6"
        request.motion_plan_request.goal_constraints.append(
            self.create_pose_constraint(target_pose)
        )

        future = self._planning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().motion_plan_response.trajectory:
            return future.result().motion_plan_response.trajectory
        else:
            self.get_logger().error("Motion planning failed")
            return None

    def send_goal(self, trajectory):
        if trajectory is None:
            self.get_logger().error("No valid trajectory to execute")
            return
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        future = goal_handle.get_result_async()
        future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().info(f'Goal failed with status: {goal_handle.status}')
    
    def get_current_pose(self):
        status_code, position = self._arm.get_position()
        x, y, z, roll, pitch, yaw = position
        return [x, y, z]

def main(args=None):
    rclpy.init(args=args)
    move_group_client = MoveGroupClient()
    current_pose = move_group_client.get_current_pose()
    current_position_x = current_pose[0]
    current_position_y = current_pose[1]
    current_position_z = current_pose[2]
    target_pose = Pose()
    target_pose.position.x = current_position_x + 0.3  # 로봇 팔이 이동할 수 있는 범위 내
    target_pose.position.y = current_position_y
    target_pose.position.z = current_position_z # 너무 높지 않게 설정

    target_pose.orientation.w = 1.0

    planned_trajectory = move_group_client.compute_motion_plan(target_pose)
    print(f'planned_trajectory:{planned_trajectory}')
    move_group_client.send_goal(planned_trajectory)

    rclpy.spin(move_group_client)
    move_group_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# class RobotMain(object):
#     """Robot Main Class"""
#     def __init__(self, robot, **kwargs):
#         self.alive = True
#         self._arm = robot
#         self._tcp_speed = 100
#         self._tcp_acc = 2000
#         self._angle_speed = 20
#         self._angle_acc = 500
#         self._vars = {}
#         self._funcs = {}
#         self._robot_init()

#     # Robot init
#     def _robot_init(self):
#         self._arm.clean_warn()
#         self._arm.clean_error()
#         self._arm.motion_enable(True)
#         self._arm.set_mode(0)
#         self._arm.set_state(0)
#         time.sleep(1)
#         self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
#         self._arm.register_state_changed_callback(self._state_changed_callback)
#         if hasattr(self._arm, 'register_count_changed_callback'):
#             self._arm.register_count_changed_callback(self._count_changed_callback)

#     # Register error/warn changed callback
#     def _error_warn_changed_callback(self, data):
#         if data and data['error_code'] != 0:
#             self.alive = False
#             self.pprint('err={}, quit'.format(data['error_code']))
#             self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

#     # Register state changed callback
#     def _state_changed_callback(self, data):
#         if data and data['state'] == 4:
#             self.alive = False
#             self.pprint('state=4, quit')
#             self._arm.release_state_changed_callback(self._state_changed_callback)

#     # Register count changed callback
#     def _count_changed_callback(self, data):
#         if self.is_alive:
#             self.pprint('counter val: {}'.format(data['count']))

#     def _check_code(self, code, label):
#         if not self.is_alive or code != 0:
#             self.alive = False
#             ret1 = self._arm.get_state()
#             ret2 = self._arm.get_err_warn_code()
#             self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
#         return self.is_alive

#     @staticmethod
#     def pprint(*args, **kwargs):
#         try:
#             stack_tuple = traceback.extract_stack(limit=2)[0]
#             print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
#         except:
#             print(*args, **kwargs)

#     @property
#     def arm(self):
#         return self._arm

#     @property
#     def VARS(self):
#         return self._vars

#     @property
#     def FUNCS(self):
#         return self._funcs

#     @property
#     def is_alive(self):
#         if self.alive and self._arm.connected and self._arm.error_code == 0:
#             if self._arm.state == 5:
#                 cnt = 0
#                 while self._arm.state == 5 and cnt < 5:
#                     cnt += 1
#                     time.sleep(0.1)
#             return self._arm.state < 4
#         else:
#             return False

#     # Robot Main Run
#     def run(self):
#         try:
#             code = self._arm.set_servo_angle(angle=[0.1, 19.7, 90.9, -1.8, -112.5, -180.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
#             if not self._check_code(code, 'set_servo_angle'):
#                 return
#         except Exception as e:
#             self.pprint('MainException: {}'.format(e))
#         self.alive = False
#         self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
#         self._arm.release_state_changed_callback(self._state_changed_callback)
#         if hasattr(self._arm, 'release_count_changed_callback'):
#             self._arm.release_count_changed_callback(self._count_changed_callback)


# if __name__ == '__main__':
#     RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
#     arm = XArmAPI('192.168.1.192', baud_checkset=False)
#     robot_main = RobotMain(arm)
#     robot_main.run()

