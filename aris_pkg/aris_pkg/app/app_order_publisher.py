#!/usr/bin/env python3
import rclpy as rp
from rclpy.node import Node

import threading
import socket
import re

from my_first_pkg_msgs.msg import AppOrder 
from std_msgs.msg import Bool

class OrderPublisher(Node):
    
    def __init__(self):
        super().__init__('order')
        self.host = "192.168.0.37"  # 서버 IP (서버가 실행되는 PC의 IP 주소)
        self.port_order = 6789  # 주문내용 받는 포트
        self.port_voice = 8888  # 음성서비스 요청 받는 포트
        self.order_publisher = self.create_publisher(AppOrder, '/app_order', 10)
        self.voice_publisher = self.create_publisher(Bool, '/voice_recognize', 10)
        self.order = AppOrder()
        self.voice_bool = Bool()
        self.server_order_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_order_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_order_socket.bind((self.host, self.port_order))  # 서버 IP와 포트 번호 바인딩
        self.server_order_socket.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        self.server_voice_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_voice_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_voice_socket.bind((self.host, self.port_voice))  # 서버 IP와 포트 번호 바인딩
        self.server_voice_socket.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        print("서버가 시작되었습니다. 클라이언트를 기다립니다...")
        self.client_order_socket=None
        self.client_order_address=None
        self.client_voice_socket=None
        self.client_voice_address=None
        self.pattern = r"Topping:\s*([^ ]+).*Cup/Cone:\s*([^ ]+)"

    def receive_and_publish_order(self):
        while True:
            try:
                # 클라이언트 연결 수락
                self.client_order_socket, self.client_order_address = self.server_order_socket.accept()
                print(f"client_order_address 값 확인: {self.client_order_address}") 
                print(f"클라이언트 {self.client_order_address} 가 연결되었습니다.")

                while True:
                    # 클라이언트로부터 메시지 수신 (최대 1024 바이트)
                    data_order = self.client_order_socket.recv(1024)
                    if data_order:
                        order_message = data_order.decode()  # 수신된 데이터를 문자열로 변환
                        print(f"받은 메시지: {order_message}")
                        match = re.search(self.pattern, order_message)                 
                        self.order.topping_type=match.group(1)
                        self.order.cup_or_cone=match.group(2)
                        self.order_publisher.publish(self.order)
                    else:
                        # 클라이언트가 연결을 끊은 경우
                        print("클라이언트가 연결을 종료했습니다.")
                        break
            except KeyboardInterrupt:
                print("\nServer interrupted by user. Closing server socket...")
                # 클라이언트 연결 종료
                self.client_order_socket.close()
                # 서버 종료 시 소켓 닫기
                self.server_order_socket.close()
                print("서버 종료")
            except Exception as e:
                print(f"에러 발생: {e}")
                break

    def receive_and_publish_bool(self):
        while True:
            try:
                # 클라이언트 연결 수락
                self.client_voice_socket, self.client_voice_address = self.server_voice_socket.accept()
                print(f"client_voice_address 값 확인: {self.client_voice_address}") 
                print(f"클라이언트 {self.client_voice_address}가 연결되었습니다.")

                while True:
                    # 클라이언트로부터 메시지 수신 (최대 1024 바이트)
                    data_voice = self.client_voice_socket.recv(1024)
                    if data_voice:
                        voice_message = data_voice.decode()  # 수신된 데이터를 문자열로 변환
                        print(f"받은 메시지: {voice_message}")
                        if voice_message=="True":
                            boolean=True
                        else:
                            boolean=False
                        self.voice_bool.data=boolean                
                        self.voice_publisher.publish(self.voice_bool)
                    else:
                        # 클라이언트가 연결을 끊은 경우
                        print("클라이언트가 연결을 종료했습니다.")
                        break
            except KeyboardInterrupt:
                print("\nServer interrupted by user. Closing server socket...")
                # 클라이언트 연결 종료
                self.client_voice_socket.close()
                # 서버 종료 시 소켓 닫기
                self.server_voice_socket.close()
                print("서버 종료")
            except Exception as e:
                print(f"에러 발생: {e}")
                break

    def thread_start(self):
        # 두 개의 스레드 생성 및 시작
            order_thread = threading.Thread(target=self.receive_and_publish_order)
            voice_thread = threading.Thread(target=self.receive_and_publish_bool)

            order_thread.start()
            voice_thread.start()

            order_thread.join()
            voice_thread.join()

            # 클라이언트 연결 종료
            self.client_order_socket.close()
            self.client_voice_socket.close()
            # 소켓 종료
            self.server_order_socket.close()
            self.server_voice_socket.close()
            print("서버 종료")

def main(args=None):
    rp.init(args=args)
    
    order_node = OrderPublisher()
    
    order_node.thread_start()
    
    order_node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
