#!/usr/bin/env python3
import rclpy as rp
from rclpy.node import Node

import threading
import socket
import re

from my_first_pkg_msgs.msg import AppOrder 
from std_msgs.msg import Bool

def get_external_ip():
    """외부 네트워크를 통해 현재 장치의 IP 주소를 가져오는 함수"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))  # Google DNS 서버로 가짜 연결
        return s.getsockname()[0]
    
class OrderPublisher(Node):
    
    def __init__(self):
        super().__init__('order')
        self.host = get_external_ip()  # 서버 IP (서버가 실행되는 PC의 IP 주소)
        self.port_list = [80,6789,6666,7777,8888,9999]  # 6789: 주문내역, 6666: gpt-api이용, 7777: 게임실행, 8888: , 9999
        self.order_publisher = self.create_publisher(AppOrder, '/app_order', 10)
        self.voice_publisher = self.create_publisher(Bool, '/voice_recognize', 10)
        self.doll_publisher = self.create_publisher(Bool, '/doll_claw', 10)
        self.order = AppOrder()
        self.voice_bool = Bool()
        self.doll_bool = Bool()
        self.server_socket_1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket_1.bind((self.host, self.port_list[1]))  # 서버 IP와 포트 번호 바인딩
        self.server_socket_1.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        self.server_socket_2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket_2.bind((self.host, self.port_list[2]))  # 서버 IP와 포트 번호 바인딩
        self.server_socket_2.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        self.server_socket_3 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_3.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket_3.bind((self.host, self.port_list[3]))  # 서버 IP와 포트 번호 바인딩
        self.server_socket_3.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        self.server_socket_4 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket_4.bind((self.host, self.port_list[4]))  # 서버 IP와 포트 번호 바인딩
        self.server_socket_4.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        self.server_socket_5 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_5.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket_5.bind((self.host, self.port_list[5]))  # 서버 IP와 포트 번호 바인딩
        self.server_socket_5.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        print("서버가 시작되었습니다. 클라이언트를 기다립니다...")
        print(f"서버IP:{self.host}")
        host_1, port_1 = self.server_socket_1.getsockname()
        host_2, port_2 = self.server_socket_2.getsockname()
        host_3, port_3 = self.server_socket_3.getsockname()
        host_4, port_4 = self.server_socket_4.getsockname()
        host_5, port_5 = self.server_socket_5.getsockname()
        print(f"서버소켓 {port_1} {port_2} {port_3} {port_4} {port_5}")
        self.client_socket_1=None
        self.client_address_1 =None
        self.client_socket_2=None
        self.client_address_2 =None
        self.client_socket_3=None
        self.client_address_3 =None
        self.client_socket_4=None
        self.client_address_4=None
        self.client_socket_5=None
        self.client_address_5 =None
        self.pattern = r"Topping:\s*([^ ]+).*Cup/Cone:\s*([^ ]+)"

    
    def receive_and_publish_order(self):
        while True:
            try:
                # 클라이언트 연결 수락
                self.client_socket_1, self.client_address_1 = self.server_socket_1.accept() 
                print(f"클라이언트 {self.client_address_1} 가 연결되었습니다.")

                while True:
                    # 클라이언트로부터 메시지 수신 (최대 1024 바이트)
                    data_order = self.client_socket_1.recv(1024)
                    if data_order:
                        order_message = data_order.decode()  # 수신된 데이터를 문자열로 변환
                        print(f"받은 메시지: {order_message}")
                        match = re.search(self.pattern, order_message)                 
                        self.order.topping_type=match.group(1)
                        self.order.cup_or_cone=match.group(2)
                        self.order_publisher.publish(self.order)
                    else:
                        # 클라이언트가 연결을 끊은 경우
                        print(f"클라이언트 {self.client_address_1}가 연결을 종료했습니다.")
                        break
            except KeyboardInterrupt:
                print("\nServer interrupted by user. Closing server socket...")
                # 클라이언트 연결 종료
                self.client_socket_1.close()
                # 서버 종료 시 소켓 닫기
                self.server_socket_1.close()
                print("서버 종료")
            except Exception as e:
                print(f"에러 발생: {e}")
                break

    def receive_and_publish_voice(self):
        while True:
            try:
                # 클라이언트 연결 수락
                self.client_socket_2, self.client_address_2  = self.server_socket_2.accept()
                print(f"클라이언트 {self.client_address_2 }가 연결되었습니다.")

                while True:
                    # 클라이언트로부터 메시지 수신 (최대 1024 바이트)
                    data= self.client_socket_2.recv(1024)
                    if data:
                        message = data.decode()  # 수신된 데이터를 문자열로 변환
                        print(f"받은 메시지: {message}")
                        if message=="True":
                            boolean=True
                        else:
                            boolean=False
                        self.voice_bool.data=boolean                
                        self.voice_publisher.publish(self.voice_bool)
                    else:
                        # 클라이언트가 연결을 끊은 경우
                        print(f"클라이언트 {self.client_address_2}가 연결을 종료했습니다.")
                        break
            except KeyboardInterrupt:
                print("\nServer interrupted by user. Closing server socket...")
                # 클라이언트 연결 종료
                self.client_socket_2.close()
                # 서버 종료 시 소켓 닫기
                self.server_socket_2.close()
                print("서버 종료")
            except Exception as e:
                print(f"에러 발생: {e}")
                break

    def receive_and_publish_doll(self):
        while True:
            try:
                # 클라이언트 연결 수락
                self.client_socket_3, self.client_address_3 = self.server_socket_3.accept()
                print(f"클라이언트 {self.client_address_3}가 연결되었습니다.")

                while True:
                    # 클라이언트로부터 메시지 수신 (최대 1024 바이트)
                    data= self.client_socket_3.recv(1024)
                    if data:
                        message = data.decode()  # 수신된 데이터를 문자열로 변환
                        print(f"받은 메시지: {message}")
                        if message=="True":
                            boolean=True
                        else:
                            boolean=False
                        self.doll_bool.data=boolean                
                        self.doll_publisher.publish(self.doll_bool)
                    else:
                        # 클라이언트가 연결을 끊은 경우
                        print(f"클라이언트 {self.client_address_3}가 연결을 종료했습니다.")
                        break
            except KeyboardInterrupt:
                print("\nServer interrupted by user. Closing server socket...")
                # 클라이언트 연결 종료
                self.client_socket_3.close()
                # 서버 종료 시 소켓 닫기
                self.server_socket_3.close()
                print("서버 종료")
            except Exception as e:
                print(f"에러 발생: {e}")
                break
            
    def thread_start(self):
        try:
            # 스레드 생성
            threads = [
                threading.Thread(target=self.receive_and_publish_order, daemon=True),
                threading.Thread(target=self.receive_and_publish_voice, daemon=True),
                threading.Thread(target=self.receive_and_publish_doll, daemon=True)
            ]

            # 실행
            for thread in threads:
                thread.start()

            # 서버 계속 실행 (join 제거)
            while True:
                rp.spin(self)
        except KeyboardInterrupt:
            print("서버 종료 중...")
        finally:
            # 모든 소켓 안전하게 닫기
            sockets = [
                self.server_socket_1, self.server_socket_2, self.server_socket_3,
                self.server_socket_4, self.server_socket_5
            ]
            for sock in sockets:
                sock.close()
            print("서버 종료 완료")

def main(args=None):
    rp.init(args=args)
    
    order_node = OrderPublisher()
    
    order_node.thread_start()
    
    order_node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()