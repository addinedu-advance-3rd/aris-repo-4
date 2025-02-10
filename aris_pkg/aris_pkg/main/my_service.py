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
from std_msgs.msg import Int64, Bool, String
from my_first_pkg_msgs.msg import AppOrder, OrderHistory
from my_first_pkg_msgs.srv import CapsulePosition

class my_service(Node):
    def __init__(self):
        super().__init__('my_service')
        self.server=self.create_client(CapsulePosition, '/process_order')
        self.request=CapsulePosition.Request()
        self.request.req=True
def main(args=None):
    rclpy.init(args=args)
    Service=my_service()
    future=Service.server.call_async(Service.request)
    print(Service.request)
    
    while not future.done():
        rclpy.spin_once(Service)
        print(future.result())
        print(future.done())
    rclpy.shutdown

if __name__=='__main__':
    main()
