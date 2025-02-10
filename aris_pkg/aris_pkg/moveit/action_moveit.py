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
from moveit_msgs.msg import (MotionPlanRequest, RobotTrajectory, Constraints,
                             PositionConstraint, OrientationConstraint, BoundingVolume)
# MoveIt2의 Cartesian path planning 서비스를 사용 (ROS 메시지 타입은 환경에 따라 다를 수 있음)
from moveit_msgs.srv import GetMotionPlan, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
# tf_transformations 대신 scipy를 사용하여 Euler -> quaternion 변환
from scipy.spatial.transform import Rotation as R

import numpy as np
from copy import deepcopy
from builtin_interfaces.msg import Duration


class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_client')
        self.arm = None

        # Action Client 생성 (trajectory 실행)
        self._action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        # MotionPlan 서비스를 위한 클라이언트 (필요시 사용)
        self._planning_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        # Cartesian path planning 서비스를 위한 클라이언트 생성
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        try:
            arm = XArmAPI('192.168.1.192', baud_checkset=False)
            self._arm = arm
            self._arm.connect()  # xArm 연결 시도
            self.get_logger().info("Connected to XArm")
        except Exception as e:
            self.get_logger().error(f"Error connecting to the arm: {str(e)}")
            return

    def compute_cartesian_path(self, waypoints, eef_step=0.001, jump_threshold=0.0):
        """
        Cartesian 경로 계획을 수행하는 함수.
        :param waypoints: 이동할 waypoint의 리스트 (Pose 객체 리스트)
        :param eef_step: 엔드 이펙터의 step 크기 (m)
        :param jump_threshold: 점프 임계값 (0.0이면 사용 안함)
        :return: (계획된 trajectory (RobotTrajectory), 성공 fraction)
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartesian path planning 서비스가 사용 가능하지 않습니다.")
            return None, 0.0

        request = GetCartesianPath.Request()
        request.group_name = "lite6"
        request.link_name = "link6"
        request.waypoints = waypoints
        request.max_step = eef_step
        request.jump_threshold = jump_threshold
        # 필요에 따라 아래 옵션도 설정할 수 있음
        # request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            response = future.result()
            self.get_logger().info(f"Cartesian path planning 완료: fraction = {response.fraction}")
            return response.solution, response.fraction
        else:
            self.get_logger().error("Cartesian path planning 실패")
            return None, 0.0

    def send_goal(self, trajectory):
        """
        계산된 trajectory를 실행하는 함수.
        """
        if trajectory is None:
            self.get_logger().error("실행할 유효한 trajectory가 없습니다.")
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
        """
        현재 xArm의 pose를 가져오는 함수.
        """
        status_code, position = self._arm.get_position()
        x, y, z, roll, pitch, yaw = position
        roll, pitch, yaw = 180.0, 90.0, 0.0
        # Pose 객체 생성 (x, y, z는 mm 단위를 m로 변환)
        pose = Pose()
        pose.position.x = x / 1000.0
        pose.position.y = y / 1000.0
        pose.position.z = z / 1000.0
        # Euler 각도를 quaternion으로 변환 (scipy 사용)
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)

        quaternion = r.as_quat()  # 반환 순서는 [x, y, z, w]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose


def main(args=None):
    rclpy.init(args=args)
    move_group_client = MoveGroupClient()

    # 현재 xArm의 Pose를 가져옴
    start_pose = move_group_client.get_current_pose()
    move_group_client.get_logger().info(f"현재 Pose: {start_pose}")

    # # Cartesian 경로 planning을 위한 waypoint 설정
    # # 예제: 현재 위치에서 x축 방향으로 0.1m (10cm) 전진하는 경로
    # waypoints = []
    # waypoints.append(deepcopy(start_pose))  # 시작점

    # target_pose = deepcopy(start_pose)
    # target_pose.position.x += 0.1  # x축 방향으로 0.1m 전진
    # waypoints.append(deepcopy(target_pose))
    
    # 현재 pose에서 x축 방향으로 0.1m 전진하는 단순 경로 대신
    # 중간 waypoint를 추가하여 3개의 waypoint로 구성
    waypoints = []
    start_pose = deepcopy(move_group_client.get_current_pose())
    waypoints.append(start_pose)

    # 중간 waypoint (x축으로 0.05m 전진)
    mid_pose = deepcopy(start_pose)
    mid_pose.position.x += 0.05
    waypoints.append(mid_pose)

    # 최종 목표 (x축으로 0.1m 전진)
    target_pose = deepcopy(start_pose)
    target_pose.position.x -= 0.1
    waypoints.append(target_pose)

    # computeCartesianPath() 호출
    trajectory, fraction = move_group_client.compute_cartesian_path(waypoints, eef_step=0.001, jump_threshold=0.0)
    print(f"trajectory:{trajectory}")
    if trajectory is not None:
        move_group_client.get_logger().info("Cartesian 경로 planning 성공, trajectory 실행합니다.")
        move_group_client.send_goal(trajectory)
    else:
        move_group_client.get_logger().error("Cartesian 경로 planning 실패, trajectory 실행하지 않습니다.")

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

