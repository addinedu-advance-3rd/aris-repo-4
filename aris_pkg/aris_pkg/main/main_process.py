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
from configparser import ConfigParser
import rclpy
from rclpy.node import Node
from aris_pkg.main.main_module import ToppingMain 
from std_msgs.msg import Int64, Bool, Int32MultiArray
from my_first_pkg_msgs.msg import OrderHistory
from my_first_pkg_msgs.msg import AppOrder

class RobotArm(Node):
    def __init__(self):
        super().__init__('aris_robot_arm')
        self.mode = 'start'
        arm = XArmAPI('192.168.1.192', baud_checkset=False)
        self.mimo  = ToppingMain(arm)
        self.timer = self.create_timer(1.0, self.modes)
        self.before_mode = 'start'
        
        # Data validation flags
        self.order_check = []
        self.capsule_check = 1
        self.picandplace = True
        self.prohibit = True
        self.capsule_check = True
        self.topping_mode = 1

        self.inventory_topping = {
            '죠리퐁': 0,
            '코코볼': 0,
            '해바라기씨': 0,
            'None': 0,
        }
        
        self.inventory_cup_cone = {
            'cup': 0,
            'cone': 0,
        }


       # 원래 False가 맞음 
        self.prohibit_flag = True
        self.order_check_flag = False
        self.capsule_check_flag = True
        self.pickandplace_flag = True
        self.capsuleholder_flag = True
        self.topping_flag = True
        self.cup_mode = 0

    #     # Subscribers
        self.prohibit_subscriber = self.create_subscription(Bool, '/robot_warning', self.prohibit_check_callback, 1)
        self.subscription = self.create_subscription(AppOrder,'/app_order', self.order_callback, 1)  # 수신할 토픽 이름self.listener_callback,

    
    #     self.order_subscriber = self.create_subscription(Int32MultiArray, '/capsule_position', self.order_callback, 10)
    #     self.capsule_subscriber = self.create_subscription(Int64, '/capsule_position', self.position_callback, 10)
    #     self.pickandplace_subscriber = self.create_subscription(bool, '/pickandplace', self.pickandplace_callback, 10)
    #     self.capsuleholder_subscriber = self.create_subscription(bool, '/caopsule_check', self.capsuleholder_callback, 10)
    #     self.topping_subscriber = self.create_subscription(Int64, '/topping_check', self.topping_check_callback, 10)

 
 
    def prohibit_check_callback(self, msg):
        print('11111111111111111111111111111')
        self.prohibit = msg.data
        if msg.data == False:
            self.prohibit_flag = False
        else:
            self.prohibit_flag = True
        
        self.prohibit_flag = False
    
    
    def order_callback(self, msg):
        print('22222222222222222222222222222222222222222222222222222222222222')
        print(msg)
        # 품목별로 수량을 업데이트
        if msg.topping_type in self.inventory_topping:
            self.inventory_topping[msg.topping_type] += 1
            
            if msg.topping_type=='죠리퐁':
                self.topping_mode = 1

            elif msg.topping_type=='코코볼':
                self.topping_mode = 2

            elif msg.topping_type=='해바라기씨':
                self.topping_mode = 3
            
            elif msg.topping_type=='None':
                self.topping_mode = 0
                
        else:
            self.get_logger().warning(f"알 수 없는 토핑: {msg.topping_type}")
        
        if msg.cup_or_cone in self.inventory_cup_cone:
            self.inventory_cup_cone[msg.cup_or_cone] += 1
            
            if msg.cup_or_cone=='cup':
                self.cup_mode = 1

            elif msg.cup_or_cone=='cone':
                self.cup_mode = 0
                
        else:
            self.get_logger().warning(f"알 수 없는 그릇: {msg.cup_or_cone}")
 
        print(f"self_topping_mode:{self.topping_mode}")
        print(f"self_cup_mode:{self.cup_mode}")
        print(f"inventory:{self.inventory_topping},{self.inventory_cup_cone}")

        self.order_check_flag = True
    # def position_callback(self, msg):
    #     self.capsule_check = msg.data
    #     self.capsule_check_flag = True

    # def pickandplace_callback(self, msg):
    #     self.picandplace = msg.data
    #     self.pickandplace_flag = True

    # def capsuleholder_callback(self, msg):
    #     self.capsule_check = msg.data
    #     self.capsuleholder_flag = True

    # def topping_check_callback(self, msg):
    #     self.topping_mode = msg.data
    #     self.topping_flag = True


    def modes(self):
        if self.mode == 'start':
            self.mode_initialize()

        elif self.mode == 'order_check':
            self.mode_order_check()
        
        elif self.mode == 'check_motion':
            self.mode_check_motion()

        elif self.mode == 'verify_grasp':
            self.mode_verify_grasp()

        elif self.mode == 'move_to_holder':
            self.mode_move_to_holder()

        elif self.mode == 'dispense_cup':
            self.mode_dispense_cup()

        elif self.mode == 'dispense_ice_cream':
            self.mode_dispense_ice_cream()

        elif self.mode == 'add_topping':
            self.mode_add_topping()

        elif self.mode == 'finalize':
            self.mode_finalize()

        elif self.mode == 'wait':
            self.mode_receive_message(self.before_mode)

        elif self.mode == 'motion_following':
            self.motion_following()




    def mode_receive_message(self, before_mode):
        self.get_logger().error(f"Message not received in mode: {before_mode}")
        self.mode = self.before_mode

    def prohibit_method(self):
        print(f"self.prohibit_flag:{self.prohibit_flag}")
        if self.prohibit_flag == False:
            self.mimo.mode_prohibit_detection()
            return

    def mode_initialize(self):
        self.prohibit_method()
                
        self.get_logger().info("초기 위치로 이동")
        self.mimo.connect_machine()
        print(f"self.cup_mode: {self.cup_mode}")
        if self.cup_mode == 1:
            self.mimo.initialization()
        else:
            self.mimo.co_initialization()
        
        self.order = []
        

        if not self.order_check_flag or not self.pickandplace_flag:
            self.get_logger().error("Required data (capsule or pick and place) not received. Returning to start.")
            self.mode = 'motion_following'
            self.before_mode = 'start'

            return

        self.mode = 'check_motion'


    def mode_order_check(self):
        if self.order_check_flag:           
            self.mode = 'check_motion'
        else:
            self.mode = 'order_check'


            

    def mode_check_motion(self):
        self.prohibit_method()
        
        self.get_logger().info("캡슐 놓여져 있는 위치 확인")
        if not self.capsule_check_flag:
            self.get_logger().error("Capsule position data not received. Cannot check motion.")
            self.mode = 'wait'
            self.before_mode = 'check_motion'
            return
        
        if self.cup_mode == 1: 
            self.mimo.dispenser_moveright()
        
        num = self.capsule_check
        if self.cup_mode == 1:
            if num == 1:
                self.mimo.grip_capsule(num)
                self.mode = 'verify_grasp'
            elif num == 2:
                self.mimo.grip_capsule(num)
                self.mode = 'verify_grasp'
            elif num == 3:
                self.mimo.grip_capsule(num)
                self.mode = 'verify_grasp'
            else:
                self.handle_motion_failure()
        else:
            if num == 1:
                self.mimo.co_grip_capsule(num)
                self.mode = 'verify_grasp'
            elif num == 2:
                self.mimo.co_grip_capsule(num)
                self.mode = 'verify_grasp'
            elif num == 3:
                self.mimo.co_grip_capsule(num)
                self.mode = 'verify_grasp'
            else:
                self.handle_motion_failure()

            
    def mode_verify_grasp(self):
        self.prohibit_method()
        
        self.get_logger().info("물체 제대로 잡았는 지 확인")            
        
        if not self.pickandplace_flag:
            self.get_logger().error("Pick and place data not received. Cannot verify grasp.")
            self.mode = 'wait'
            self.before_mode = 'verify_grasp'
            return

        success = self.picandplace
        success = True  
        if success:
            self.mode = 'move_to_holder'
        else:
            self.handle_grasp_failure()

    def mode_move_to_holder(self):
        self.prohibit_method()
        
        self.get_logger().info("캡슐 홀더로 이동")
        if not self.capsuleholder_flag:
            self.get_logger().error("Capsule holder data not received. Cannot move to holder.")
            self.mode = 'wait'
            self.before_mode = 'move_to_holder'
            return
        
        if self.cup_mode == 1:
            self.mimo.pick_up_zone_to_holder()
            self.mimo.put_capsule()
            self.mimo.dispenser_moveright()
            success = self.capsule_check 
            success = True  
        
        else:
            self.mimo.co_pick_up_zone_to_holder()    
            self.mimo.co_put_capsule()    
            success = self.capsule_check 
            success = True  
        

        if success:
            self.mode = 'dispense_cup'
        else:
            self.handle_move_failure()

    def mode_dispense_cup(self):
        self.prohibit_method()

        self.get_logger().info("컵 디스펜서로 이동")
        
        if self.cup_mode == 1:
            self.mimo.holder_to_cup_dispensor()
            self.mimo.grip_cup()
            success = True 
        else:
            self.mimo.co_holder_to_cone_tray()
            self.mimo.co_grip_cone_tray()
            self.mimo.co_move_to_get_cone()      
            success = True 
                        
        if success:
            self.mode = 'dispense_ice_cream'
        else:
            self.handle_dispense_failure()

    def mode_dispense_ice_cream(self):
        self.prohibit_method()

        self.get_logger().info("아이스크림 디스펜서로 이동")
        if self.cup_mode == 1:
            self.mimo.cup_dispensor_to_ice_cream_receiver()
            self.mimo.press_ice_cream()
            self.mimo.receive_ice_cream()
            success = True  
        else:
            self.mimo.co_to_ice_cream_receiver()   
            self.mimo.co_receive_ice_cream() 
            success = True 
        
        if success:
            self.mode = 'add_topping'
        else:
            self.handle_ice_cream_failure()

    def mode_add_topping(self):
        self.prohibit_method()
        
        self.get_logger().info("아이스크림 위에 토핑 받기")
        if not self.topping_flag:
            self.get_logger().error("Topping data not received. Cannot add topping.")
            self.mode = 'wait'
            self.before_mode = 'add_topping'
            return

        if self.cup_mode == 1:  
        # print(f"topping_mode:{topping_mode}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.mimo.receive_topping(self.topping_mode)
        
        else:
            self.mimo.co_receive_topping(self.topping_mode)

        success = True  
        if success:
            self.mode = 'finalize'        
        else:
            self.handle_topping_failure()

    def mode_finalize(self):
        self.prohibit_method()
        
        self.get_logger().info("Process complete. Returning to initial position...")
        if self.cup_mode == 1:         
            self.mimo.to_pick_up_zone()

            self.mimo.pick_up_zone_to_holder()
            self.mimo.grip_empty_capsule() 
            self.mimo.drop_empty_capsule()
            self.mimo.finish_process()
        else:
            self.mimo.co_give_ice_cream_cone()
            
            
            self.mimo.co_place_cone_tray()        
            self.mimo.co_pick_up_zone_to_holder() 
            self.mimo.co_grip_empty_capsule()  
            self.mimo.co_drop_empty_capsule()
            self.mimo.co_finish_process()            

        self.mode = 'start'
        self.order_check_flag = False
        self.capsule_check_flag = True
        self.pickandplace_flag = True
        self.capsuleholder_flag = False
        self.topping_flag = False
        self.topping_mode = 1
        self.cup_mode = 0

    def motion_following(self):
        print('와 신난다')
        num = self.capsule_check
        self.mode = 'order_check'
        # if num in (1, 2, 3):
        #     self.mode = 'check_motion'

        
    # Control and Feedback Logic for Failures
    def handle_motion_failure(self):
        self.get_logger().error("Motion failure detected. Re-attempting motion...")
        self.mimo.initialization()  # Retry initialization
        self.mode = 'motion_following'

    def handle_grasp_failure(self):
        self.get_logger().error("Grasp failure detected. Adjusting position and retrying...")
        #self.mimo.grip_capsule()  # Retry gripping the capsule
        self.mode = 'verify_grasp'

    def handle_move_failure(self):
        self.get_logger().error("Move failure detected. Recalculating path...")
        #self.mimo.pick_up_zone_to_holder()  # Retry moving to holder
        self.mode = 'move_to_holder'

    def handle_dispense_failure(self):
        self.get_logger().error("Cup dispensing failure detected. Checking dispenser...")
        #self.mimo.holder_to_cup_dispensor()  # Retry dispensing cup
        self.mode = 'dispense_cup'

    def handle_ice_cream_failure(self):
        self.get_logger().error("Ice cream dispensing failure detected. Adjusting dispenser...")
        #self.mimo.press_ice_cream()  # Retry dispensing ice cream
        self.mode = 'dispense_ice_cream'

    def handle_topping_failure(self):
        self.get_logger().error("Topping failure detected. Reattempting topping addition...")
        #self.mimo.receive_topping()  # Retry adding topping
        self.mode = 'add_topping'

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    robot_arm = RobotArm()  # RobotArm 객체 생성
    rclpy.spin(robot_arm)  # 노드 실행

    # 종료 시 clean-up
    robot_arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()