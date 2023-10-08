# Arduion에서 스위치 입력이 발생하면 Serial 통신을 통해 라즈베리파이에 전달
# (v)라즈베리파이는 메시지 확인 후 터틀봇에 토픽(서비스 client) 발행
# 터틀봇은 토픽(service server) 구독 및 GPIO 제어

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
# from rclpy.qos import QoSDurabilityPolicy
# from rclpy.qos import QoSHistoryPolicy
# from rclpy.qos import QoSReliabilityPolicy

import serial
import time

from std_srvs.srv import SetBool # /opt/ros/foxy/share/std_srvs/srv

# SetBool
# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered service
# string message # informational, e.g. for error messages


class ArduinoComm(Node):
    def __init__(self):
        super().__init__('arduino_comm')
        port = '/dev/ttyACM0'  # 시리얼 포트 이름 (Linux 기준)
        baudrate = 115200
        self.serial_=serial.Serial(port,baudrate,timeout=1)
        self.timer=self.create_timer(0.001,self.timer_callback)

        # self.qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_ALL,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # self.client=self.create_client(SetBool,'go_ext_devices',qos_profile=self.qos_profile)
        self.client=self.create_client(SetBool,'go_ext_devices')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # self.publisher_ = self.create_publisher(String, 'go_ext_devices', 10)
        # self.counter=0
        
    # Timer를 이용해서 Serial Buffer 모니터(pooling -> event 변경 필요)
    def timer_callback(self):    
        if self.serial_.in_waiting > 0:
            received_data = self.serial_.readline().decode().strip() # 'utf-8'

            if received_data.split('(')[0] == 'ON': # Arduino에 연결된 입력이 Toggle(ON)된 경우
                self.get_logger().info('Received from Arduino: %s' % received_data)
                self.call_service(True) # Robot GPIO Service Call
                self.serial_.write(str.encode('OK\n')) # Arduino에 수신완료 송신(self.serial_.write(b'OK\r\n'))
                # Robot(Service Server)으로 요청하는 서비스 코드 개발이 어려울 경우 토픽 발행
                # response_msg = String()
                # response_msg.data = received_data
                # self.publisher_.publish(response_msg)
            else:
                self.get_logger().info(f'Wrong Message Received:{received_data}')

    # Service Requset
    def call_service(self, data):
        # docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#filed-types
        req=SetBool.Request()
        req.data=data
        
        future = self.client.call_async(req) # 반드시 call_async
        future.add_done_callback(self.service_callback)

    # Service Result
    def service_callback(self, future):
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Received from Service_Server(robotGPIO): {future.result().success}')
            else:
                self.get_logger().warn(f'Received from Service_Server(robotGPIO): {future.result().message}')
        else:
            self.get_logger().error(f'Service call failed')


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoComm()

    try:
        # node.call_service(True) # Test Code
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        node.serial_.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()