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

import random

class ToppingMain(object):
    """로봇 메인 클래스"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self.a = random.randint(1,3) #캡슐 존재 파악 로직 필요
        self.b = random.randint(1,8) #토핑 옵션
        self._arm = robot  # XArm 객체 초기화
        self._tcp_speed = 100  # TCP 속도 설정
        self._tcp_acc = 2000  # TCP 가속도 설정
        self._angle_speed = 50  # 관절 속도 설정
        self._angle_acc = 500  # 관절 가속도 설정
        self._angle_speed_low = 10  
        self._vars = {}
        self._funcs = {}
        self._robot_init()



    def _robot_init(self):
        """로봇 초기화"""
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)  # 모드 설정
        self._arm.set_state(0)  # 상태 설정
        time.sleep(1)  # 초기화 대기 시간
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    def _error_warn_changed_callback(self, data):
        """오류/경고 변경 콜백 함수"""
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    def _state_changed_callback(self, data):
        """상태 변경 콜백 함수"""
        if data and data['state'] == 4:  # 상태가 4일 경우 종료
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _count_changed_callback(self, data):
        """카운트 변경 콜백 함수"""
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        """명령어 실행 결과 코드 확인"""
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}, ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        """로그 출력 함수"""
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        """로봇 상태 확인"""
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:  # 상태가 5일 경우 최대 0.5초 대기
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def connect_machine(self):
        """로봇 연결"""
        ip = '192.168.1.192'
        self._arm = XArmAPI(ip)  # 로봇 객체 생성
        time.sleep(0.5)
        if self._arm.warn_code != 0:
            self._arm.clean_warn()
        if self._arm.error_code != 0:
            self._arm.clean_error()

    def initialization(self):
        # 그리퍼 열기(초기화)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        # motion plaaning으로 홈으로가는 로직 필요

        # Home Sweet Home
        code = self._arm.set_servo_angle(angle=[180, -15, 25, 180, 50, 0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def dispenser_moveright(self):
        # 컵 디스팬서 끄기(초기화)
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return            
        
        # 컵 디스팬서 우로이동(컵 꺼낼 준비)
        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
    def grip_capsule(self, a):
        # 캡슐 위치옵션(1:1번캡슐, 2:2번캡슐, 3:3번캡슐)
        # 1번 캡슐 잡고 들어올리기
        if a==1:
            code = self._arm.set_servo_angle(angle=[174.1, 47.2, 67.7, 84.5, 92, 20.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[191.3, 48.1, 69.2, 100.6, 85.9, 20.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[191.3, 34.6, 68.2, 99.5, 83.7, 33.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        # 2번 캡슐 잡고 들어올리기
        elif a==2:
            code = self._arm.set_servo_angle(angle=[169.3, 25.1, 24.4, 79.4, 89.8, -0.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[200.2, 26.2, 26.6, 110.3, 89.8, 0.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[200.2, 2.6, 25.2, 108.8, 82.3, 21.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        # 3번 캡슐 잡고 들어올리기        
        elif a==3:
            code = self._arm.set_servo_angle(angle=[180.2, 29.5, 33.1, 53.7, 92.1, 2.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[218.7, 24.0, 22.1, 92.2, 90.1, -1.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[218.7, -2.0, 20.6, 92.0, 89.2, 22.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
                
    def pick_up_zone_to_holder(self):  # 캡슐홀더로 이동
        code = self._arm.set_servo_angle(angle=[164.7, -5.7, 8.9, 90.9, 89.8, 14.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[0.0, -5.4, 9.2, 90.0, 90.0, 14.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[1.4, -50.1, 11.7, 182.9, 28.2, -2.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -40.2, 49.1, 92.3, 18.1, 87.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def put_capsule(self):  # 캡슐 넣고 나오기       
        code = self._arm.set_servo_angle(angle=[17.4, 3.5, 81.4, 93.8, 73, 77.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[17.4, 3, 77.3, 94.8, 73.3, 73.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return    
        code = self._arm.set_pause_time(0.5)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -41.7, 44.6, 101.1, 18.5, 78.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def holder_to_cup_dispensor(self): # 컵 잡는 곳으로 이동            
        code = self._arm.set_servo_angle(angle=[-10.9, 22.0, 49.5, 102.6, -96.6, -26.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return    
        code = self._arm.set_servo_angle(angle=[-10.9, 52.0, 55.2, 104.1, -90.8, -3.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def grip_cup(self):
        # 컵디스팬서 좌로이동(컵꺼내짐)    
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_pause_time(5)
        if not self._check_code(code, 'set_pause_time'):
            return
        # 컵 잡고 위로 나오기     
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        code = self._arm.set_pause_time(2)
        if not self._check_code(code, 'set_pause_time'):
            return 
        code = self._arm.set_servo_angle(angle=[-10.9, 22.0, 49.5, 102.6, -96.6, -26.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def cup_dispensor_to_ice_cream_receiver(self): 

        #아이스크림 받는 곳으로 이동            
        code = self._arm.set_servo_angle(angle=[-4.7, -34.7, 31.7, 96.2, 2.5, -6.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_servo_angle(angle=[16.4, 16.6, 52.1, 103.4, 80.6, 34.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_pause_time(1)
        if not self._check_code(code, 'set_pause_time'):
            return
        
    def press_ice_cream(self):

        # 프레스기 ON       
        code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        code = self._arm.set_pause_time(15)
        if not self._check_code(code, 'set_pause_time'):
            return
        # 프레스기 OFF    
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
           
    def receive_ice_cream(self): # 정밀한 제어 필요 고려
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return 
        # 몸통 접기         
        code = self._arm.set_servo_angle(angle=[30.0, -24.5, 10.9, 90.0, 90.0, 35.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return      
        
        
    def receive_topping(self, b):
        
        # 토핑 옵션(1:선택안함, 2:0번, 3:1번, 4:2번)
        
        # 토핑옵션 존재할시 중립위치로 이동
        if b!=0:   
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        if b==1:    
            #0번 토핑기 위치로 가서 토핑 받고 중립위치 이동     
            code = self._arm.set_servo_angle(angle=[131.3, 11.4, 33.1, 165.4, 69.0, 5.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return     
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return 
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return        

        elif b==2:                    
            #1번 토핑기 위치로 가서 토핑 받고 중립위치 이동   
            code = self._arm.set_servo_angle(angle=[131.0, -12.5, 9.5, 213.0, 71.3, -11.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return         
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return     
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return             
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return     

        elif b==3:    
            # 2번 토핑기 위치로 가서 토핑 받고 중립위치 이동    
            code = self._arm.set_servo_angle(angle=[62.4, -22.9, 2.8, 149.8, 67.4, 12.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return     
            code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return              
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return       

    def to_pick_up_zone(self):    
        # 아이스크림 놓으러 이동
        code = self._arm.set_servo_angle(angle=[150.0, 0.9, 8.6, 90.0, 90.0, 7.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
            
        # 1번에  놓고 나오기 
        if self.a==1:
            code = self._arm.set_servo_angle(angle=[174.1, 47.5, 67.5, 84.4, 92.1, 19.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[191.2, 48.3, 69.1, 100.5, 86.1, 20.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            
            code = self._arm.set_servo_angle(angle=[174.1, 47.5, 67.5, 84.4, 92.1, 19.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
        # 2번에 놓고 나오기    
        elif self.a==2:
            code = self._arm.set_servo_angle(angle=[169.3, 25.1, 24.4, 79.4, 89.8, -0.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[200.2, 26.2, 26.6, 110.3, 89.8, 0.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return

            code = self._arm.set_servo_angle(angle=[169.3, 25.1, 24.4, 79.4, 89.8, -0.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        # 3번에 놓고 나오기        
        elif self.a==3:
            code = self._arm.set_servo_angle(angle=[180.2, 29.5, 33.1, 53.7, 92.1, 2.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[218.7, 24.0, 22.1, 92.2, 90.1, -1.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return

            code = self._arm.set_servo_angle(angle=[180.2, 29.5, 33.1, 53.7, 92.1, 2.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return 
        
    def grip_empty_capsule(self):  # 캡슐 넣고 나오기       
        code = self._arm.set_servo_angle(angle=[17.4, 3, 77.3, 94.8, 73.3, 73.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return    
        code = self._arm.set_pause_time(0.5)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_servo_angle(angle=[17.4, 3.5, 81.4, 93.8, 73, 77.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -41.7, 44.6, 101.1, 18.5, 78.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def drop_empty_capsule(self):
        code = self._arm.set_servo_angle(angle=[-2.6, -42.3, 44.3, 95.2, 32.9, 83.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -44.5, 14.4, 147.7, 35.6, 27.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[32.8, 6.7, 15.1, 122.6, 85.5, 7.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[32.8, 6.7, 15.1, 122.6, 85.5, -172.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[32.8, 6.7, 15.1, 122.6, 85.5, 7.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return 
          
    def finish_process(self): 
        code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return  
        # Home Sweet Home    
        code = self._arm.set_servo_angle(angle=[180.0, -15.0, 25.0, 180.0, 50.0, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
            
        # gripper off    
        code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
        if not self._check_code(code, 'set_tgpio_digital'):
            return

    def co_initialization(self):
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[180.0, -15.0, 25.0, 180.0, 50.0, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def co_grip_capsule(self, a):
        # 캡슐 위치옵션(1:1번캡슐, 2:2번캡슐, 3:3번캡슐)
        # 1번 캡슐 잡고 들어올리기
        if a==1:
            code = self._arm.set_servo_angle(angle=[174.1, 47.2, 67.7, 84.5, 92, 20.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[191.3, 48.1, 69.2, 100.6, 85.9, 20.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[191.3, 34.6, 68.2, 99.5, 83.7, 33.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        # 2번 캡슐 잡고 들어올리기
        elif a==2:
            code = self._arm.set_servo_angle(angle=[169.3, 25.1, 24.4, 79.4, 89.8, -0.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[200.2, 26.2, 26.6, 110.3, 89.8, 0.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[200.2, 2.6, 25.2, 108.8, 82.3, 21.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        # 3번 캡슐 잡고 들어올리기        
        elif a==3:
            code = self._arm.set_servo_angle(angle=[180.2, 29.5, 33.1, 53.7, 92.1, 2.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[218.7, 24.0, 22.1, 92.2, 90.1, -1.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(0.5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[218.7, -2.0, 20.6, 92.0, 89.2, 22.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
    
    def co_pick_up_zone_to_holder(self):  # 캡슐홀더로 이동
        code = self._arm.set_servo_angle(angle=[164.7, -5.7, 8.9, 90.9, 89.8, 14.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[0.0, -5.4, 9.2, 90.0, 90.0, 14.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[1.4, -50.1, 11.7, 182.9, 28.2, -2.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -40.2, 49.1, 92.3, 18.1, 87.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def co_put_capsule(self):  # 캡슐 넣고 나오기       
        code = self._arm.set_servo_angle(angle=[17.4, 3.5, 81.4, 93.8, 73, 77.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[17.4, 3, 77.3, 94.8, 73.3, 73.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return    
        code = self._arm.set_pause_time(0.5)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -41.7, 44.6, 101.1, 18.5, 78.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def co_holder_to_cone_tray(self):
        code = self._arm.set_servo_angle(angle=[-6.3, 35.1, 69.3, 84.8, -86.4, -34.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[-6.3, 59.3, 74.3, 83.9, -88.3, -14.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return 
           
    def co_grip_cone_tray(self):
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_pause_time(2)
        if not self._check_code(code, 'set_pause_time'):
            return     
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        code = self._arm.set_pause_time(2)
        if not self._check_code(code, 'set_pause_time'):
            return 
        code = self._arm.set_servo_angle(angle=[-6.3, 35.1, 69.3, 84.8, -86.4, -34.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def co_move_to_get_cone(self):
        code = self._arm.set_servo_angle(angle=[-40.0, -20.0, 31.7, 96.2, 2.5, -6.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[-90.0, -31.8, 14.8, 90.0, 90.0, 46.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[-160.0, -14.7, 33.5, 103.7, 75.0, 46.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return  
        #콘 올려놓을때까지 기다리는 로직 필요
        code = self._arm.set_pause_time(5)
        if not self._check_code(code, 'set_pause_time'):
            return   
 
    
    def co_to_ice_cream_receiver(self):
        code = self._arm.set_servo_angle(angle=[0.0, 0.9, 8.6, 90.0, 90.0, 7.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[16.4, 11.5, 55.2, 101.9, 78.8, 42.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return 
    
    def co_receive_ice_cream(self):
        
        # 프레스기 ON       
        code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        code = self._arm.set_pause_time(11)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_servo_angle(angle=[16.4, 18.3, 51.4, 103.8, 81.2, 32.1], speed=self._angle_speed_low, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_pause_time(4)
        if not self._check_code(code, 'set_pause_time'):
            return
        # 프레스기 OFF    
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return 
        code = self._arm.set_servo_angle(angle=[30.0, -24.5, 10.9, 90.0, 90.0, 35.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def co_receive_topping(self, b):
        
        # 토핑 옵션(1:선택안함, 2:0번, 3:1번, 4:2번)
        
        # 토핑옵션 존재할시 중립위치로 이동
        if b!=0:   
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        if b==1:    
            #0번 토핑기 위치로 가서 토핑 받고 중립위치 이동     
            code = self._arm.set_servo_angle(angle=[131.3, 11.4, 33.1, 165.4, 69.0, 5.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return     
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return 
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return        

        elif b==2:                    
            #1번 토핑기 위치로 가서 토핑 받고 중립위치 이동   
            code = self._arm.set_servo_angle(angle=[131.0, -12.5, 9.5, 213.0, 71.3, -11.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return         
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return     
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return             
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return     

        elif b==3:    
            # 2번 토핑기 위치로 가서 토핑 받고 중립위치 이동    
            code = self._arm.set_servo_angle(angle=[62.4, -22.9, 2.8, 149.8, 67.4, 12.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return     
            code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return              
            code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return    
    
    def co_give_ice_cream_cone(self):
        code = self._arm.set_servo_angle(angle=[200.0, -14.7, 33.5, 103.7, 75.0, 46.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        #콘 가져갈때까지 기다리는 로직 필요
        code = self._arm.set_pause_time(5)
        if not self._check_code(code, 'set_pause_time'):
            return  
               
    def co_place_cone_tray(self):
        code = self._arm.set_servo_angle(angle=[270.0, -31.8, 14.8, 90.0, 90.0, 46.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[320.0, -20.0, 31.7, 96.2, 2.5, -6.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[353.7, 35.1, 69.3, 84.8, -86.4, -34.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[353.7, 59.3, 74.3, 83.9, -88.3, -14.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_pause_time(1)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_pause_time(1)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_servo_angle(angle=[353.7, 35.1, 69.3, 84.8, -86.4, -34.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[320.0, -20.0, 31.7, 96.2, 2.5, -6.7], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[270.0, -31.8, 14.8, 90.0, 90.0, 46.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
    def co_grip_empty_capsule(self):  # 캡슐 넣고 나오기       
        code = self._arm.set_servo_angle(angle=[17.4, 3, 77.3, 94.8, 73.3, 73.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return    
        code = self._arm.set_pause_time(0.5)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_servo_angle(angle=[17.4, 3.5, 81.4, 93.8, 73, 77.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -41.7, 44.6, 101.1, 18.5, 78.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def co_drop_empty_capsule(self):
        code = self._arm.set_servo_angle(angle=[-2.6, -42.3, 44.3, 95.2, 32.9, 83.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[12.0, -44.5, 14.4, 147.7, 35.6, 27.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[32.8, 6.7, 15.1, 122.6, 85.5, 7.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[32.8, 6.7, 15.1, 122.6, 85.5, -172.9], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[32.8, 6.7, 15.1, 122.6, 85.5, 7.1], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def co_finish_process(self): 
        code = self._arm.set_servo_angle(angle=[90.0, -13.5, 8.7, 90.0, 90.0, 22.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return  
        # Home Sweet Home    
        code = self._arm.set_servo_angle(angle=[180.0, -15.0, 25.0, 180.0, 50.0, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
            
        # gripper off    
        code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
        if not self._check_code(code, 'set_tgpio_digital'):
            return
                   
    # Robot Main Run
    def run(self):
        try:
            self.connect_machine()
            print(self.a,self.b)
            
            self.initialization()

            self.grip_capsule(1)
            
            self.pick_up_zone_to_holder()
                
            self.put_capsule()

            self.holder_to_cup_dispensor()

            self.grip_cup()   

            self.cup_dispensor_to_ice_cream_receiver()   
            
            self.press_ice_cream()
            
            self.receive_ice_cream() 
            
            self.receive_topping(1)

            self.to_pick_up_zone()    
            
            self.pick_up_zone_to_holder() 
            
            self.grip_empty_capsule()  
            
            self.drop_empty_capsule()
                
            self.finish_process()
            

            
            # self.co_initialization()

            # self.co_grip_capsule(1)
            
            # self.co_pick_up_zone_to_holder()
                
            # self.co_put_capsule()

            # self.co_holder_to_cone_tray()

            # self.co_grip_cone_tray()
            
            # self.co_move_to_get_cone()
            

            # self.co_to_ice_cream_receiver()   
            
            # self.co_receive_ice_cream() 
            
            # self.co_receive_topping(1)
            
            # self.co_give_ice_cream_cone()
            
            
            # self.co_place_cone_tray()    
            
            # self.co_pick_up_zone_to_holder() 
            
            # self.co_grip_empty_capsule()  
            
            # self.co_drop_empty_capsule()
                
            # self.co_finish_process()
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    ToppingMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.192', baud_checkset=False)
    robot_main = ToppingMain(arm)
    robot_main.run()
    











# debugging

    # # Helper Methods
    # def connect_machine(self):
    #     self.get_logger().info("Connecting to the machine...")

    # def initialization(self):
    #     self.get_logger().info("Initializing system and calibrating sensors...")

    # def grip_capsule(self):
    #     self.get_logger().info("Gripping the capsule...")

    # def pick_up_zone_to_holder(self):
    #     self.get_logger().info("Moving from pick-up zone to holder...")

    # def put_capsule(self):
    #     self.get_logger().info("Placing capsule in the holder...")

    # def holder_to_cup_dispensor(self):
    #     self.get_logger().info("Moving from holder to cup dispenser...")

    # def grip_cup(self):
    #     self.get_logger().info("Gripping the cup...")

    # def cup_dispensor_to_ice_cream_receiver(self):
    #     self.get_logger().info("Moving from cup dispenser to ice cream receiver...")

    # def press_ice_cream(self):
    #     self.get_logger().info("Pressing ice cream into the cup...")

    # def receive_ice_cream(self):
    #     self.get_logger().info("Receiving ice cream...")

    # def receive_topping(self):
    #     self.get_logger().info("Receiving topping...")

    # def to_pick_up_zone(self):
    #     self.get_logger().info("Returning to pick-up zone...")

    # def finish_process(self):
    #     self.get_logger().info("Finishing process and resetting system...")