import time
import traceback
from xarm.wrapper import XArmAPI
import rclpy
from rclpy.node import Node
from my_first_pkg_msgs.msg import Delta  # 커스텀 ROS 2 메시지
import copy

class Follow(Node):
    """로봇 메인 클래스"""
    def __init__(self, robot, **kwargs):
        super().__init__('robot_follower')  # ROS 2 노드 초기화
        self.alive = True
        self._arm = robot  # XArm 객체 초기화
        self._tcp_speed = 100  # TCP 속도 설정ㅁ
        self._tcp_acc = 2000  # TCP 가속도 설정
        self._angle_speed = 20  # 관절 속도 설정
        self._angle_acc = 500  # 관절 가속도 설정
        self.delta_x = 0  # x축 이동값
        self.delta_y = 0  # y축 이동값
        self.distance = 0
        self.start_flag = True
        self.current_position = []
        # ROS 2 토픽 구독
        self.create_subscription(Delta, '/motion_topic', self.motion_callback, 10)
        
        # 로봇 초기화
        self._robot_init()

    def motion_callback(self, msg: Delta):
        """ROS 2 메시지 콜백 함수"""
        self.delta_x = msg.x  # delta_x 값 업데이트
        self.delta_y = msg.y  # delta_y 값 업데이트
        self.distance = msg.dis # distance update
        self.get_logger().info(f"Received delta_x: {self.delta_x}, delta_y: {self.delta_y}, distance : {self.distance}")  # f-string 수정
        self.move_robot()  # 메시지 수신 후 로봇 이동 함수 호출


    def set_direction(self):
        leftright_flag, updown_flag, distance_flag = 0,0,0
        leftright = self.delta_x  # delta_x 값 업데이트
        updown = self.delta_y  # delta_y 값 업데이트
        dis = self.distance
        if leftright > 5:
            leftright_flag = -1
        elif leftright < -5:
            leftright_flag  = 1
        elif updown > 5:
            updown_flag= 1
        elif updown < -5:
            updown_flag = -1 
        
        if dis > 50:
            distance_flag = 1
        return leftright_flag, updown_flag, distance_flag


    # def move_robot(self):
    #     """로봇을 delta 값에 따라 움직이는 함수"""
    #     if not self.is_alive:
    #         self.get_logger().warn("Robot is not alive. Aborting move.")
    #         return
    #     leftright, updown, gripper =  self.set_direction()
        
    #     # a = self._arm.get_position()
    #     # print(f"position:{a}")
    #     # print(f"left : {leftright} , right: {updown}")
    #     # print(f"len:{len(a[1])}")   
    #     # b, angle = self._arm.get_inverse_kinematics(a[1], input_is_radian=None, return_is_radian=None)
    #     # print(f"angle:{angle}")

    #     # b, angle = self._arm.get_joint_states()  
    #     # print(f"angle:{angle}")

        
    #     # # Apply range constraints to delta_x and delta_y
    #     # self.delta_x = max(-10, min(10, self.delta_x))
    #     # self.delta_y = max(-10, min(10, self.delta_y))

    #     # You can also apply constraints to z if needed (e.g., for z axis, set delta_z to a similar value)

    #     # code = self._arm.set_position(*[leftright, updown, 0.0, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
    #     # if not self._check_code(code, 'set_init_point'):
    #     #     return


    #     print(f"gripper: {gripper}")
    #     if gripper == 1:
    #         code = self._arm.open_lite6_gripper()
    #         #self._arm.set_pause_time(2) 
        
    #     elif gripper == 0:
    #         code = self._arm.close_lite6_gripper()
    #         # self._arm.set_pause_time(2)           

    #     code = self._arm.set_position(x=leftright*5 ,z=updown*5, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True, wait=False)
    #     if not self._check_code(code, 'set_position'):
    #        return
       

    def move_robot(self):
        """로봇을 delta 값에 따라 움직이는 함수"""
        if not self.is_alive:
            self.get_logger().warn("Robot is not alive. Aborting move.")
            return
        leftright, updown, gripper =  self.set_direction()


        print(f"gripper: {gripper}")
        if gripper == 1:
            code = self._arm.open_lite6_gripper()
            #self._arm.set_pause_time(2) 
        
        elif gripper == 0:
            code = self._arm.close_lite6_gripper()
            # self._arm.set_pause_time(2)           
        if self.start_flag == True:
            ret1, self.current_position = self._arm.get_position(is_radian=None)
            self.start_flag = False
        future_position = copy.deepcopy(self.current_position)
        
        future_position[0] += leftright*5 
        future_position[2] += updown*5
        print("current_position", self.current_position)
        print("future_position", future_position)

        #ret2, future_angle = self._arm.get_inverse_kinematics(future_position, input_is_radian=None, return_is_radian=None)

        # code = self._arm.set_servo_angle(angle=future_angle, speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
        # self.current_position = future_position
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        # code = self._arm.set_position(x=leftright*5 ,z=updown*5, radius=1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True, wait=False)
        # if not self._check_code(code, 'set_position'):
        #    return


        # code = self._arm.set_position_aa(x=future_position[0],y=future_position[1],z=future_position[2], roll=future_position[3], pitch=future_position[4],  yaw=future_position[5], radius=1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=False, wait=False)
        # self.current_position = future_position
        # if not self._check_code(code, 'set_position'):
        #    return

        code = self._arm.set_position_aa(future_position, radius=1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=False, wait=False)
        self.current_position = future_position
        if not self._check_code(code, 'set_position'):
           return

    def _robot_init(self):
        """로봇 초기화 설정"""
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)  # 기본 모드
        self._arm.set_state(0)  # 기본 상태
        time.sleep(1)  # 초기화 대기

        # 로봇 상태 변경 콜백 등록
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)

    def _error_warn_changed_callback(self, data):
        """오류/경고 변경 콜백 함수"""
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint(f"Error: {data['error_code']}, shutting down.")
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    def _state_changed_callback(self, data):
        """로봇 상태 변경 콜백 함수"""
        if data and data['state'] == 4:  # 상태가 4일 경우 종료
            self.alive = False
            self.pprint("Robot state=4, shutting down.")
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _check_code(self, code, label):
        """명령 실행 결과 확인"""
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint(f"{label}, code={code}, connected={self._arm.connected}, state={self._arm.state}, error={self._arm.error_code}, ret1={ret1}, ret2={ret2}")
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        """로그 출력 함수"""
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print(f"[{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))}][{stack_tuple[1]}] {' '.join(map(str, args))}")
        except Exception as e:
            print(*args, **kwargs)

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

def main():
    """메인 실행 함수"""
    print("Finger control node started")
    rclpy.init()
    robot = XArmAPI('192.168.1.192')  # 로봇 IP 설정
    node = Follow(robot)  # Follow 클래스 초기화

    try:
        rclpy.spin(node)  # ROS 2 이벤트 루프 처리

    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()  # 노드 종료

if __name__ == "__main__":
    main()


# import rclpy
# from rclpy.node import Node
# from my_first_pkg_msgs.msg import Delta  # Delta 메시지 임포트

# class CallbackTester(Node):
#     def __init__(self):
#         super().__init__('callback_tester')  # ROS 2 노드 초기화
#         self.create_subscription(Delta, '/motion_topic', self.motion_callback, 10)  # '/motion_topic' 구독
#         self.get_logger().info("CallbackTester node started, waiting for messages...")

#     def motion_callback(self, msg: Delta):
#         """콜백 함수: 메시지를 받으면 호출됨"""
#         self.get_logger().info(f"Received message: delta_x={msg.x}, delta_y={msg.y}")  # 메시지 값 출력

# def main():
#     """메인 실행 함수"""
#     rclpy.init()  # ROS 2 초기화
#     node = CallbackTester()  # CallbackTester 클래스 인스턴스 생성
#     try:
#         rclpy.spin(node)  # ROS 2 이벤트 루프 실행
#     except KeyboardInterrupt:
#         node.get_logger().info("Node interrupted by user.")
#     finally:
#         node.destroy_node()  # 노드 종료

# if __name__ == "__main__":
#     main()
