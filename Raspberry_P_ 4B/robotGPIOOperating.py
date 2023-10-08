# Arduion에서 스위치 입력이 발생하면 Serial 통신을 통해 라즈베리파이에 전달
# 라즈베리파이는 메시지 확인 후 터틀봇에 토픽(서비스 client) 발행
# (v)터틀봇은 토픽(service server) 구독 및 GPIO 제어

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from rclpy.qos import QoSDurabilityPolicy
# from rclpy.qos import QoSHistoryPolicy
# from rclpy.qos import QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String

import numpy as np
import math
from math import sqrt
import time
import RPi.GPIO as GPIO

from std_srvs.srv import SetBool

from rclpy.executors import MultiThreadedExecutor



class RobotGPIOHandling(Node):
    def __init__(self):
        super().__init__('robot_GPIO_handling')
        # self.qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_ALL,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_service(SetBool,
                            'go_ext_devices', 
                            self.control_gpio,callback_group=ReentrantCallbackGroup()) 
        # self.create_subscription(String, 'go_ext_devices', self.serial_send_callback, 10)
        self.pub=self.create_publisher(String, 'robot_LED_control',10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(21, GPIO.OUT)
        self.get_logger().info(f'Start Service Server')

    # service(server) - Robot에 연결된 외부 장치(GPIO) 동작
    def control_gpio(self, request, response):#request,response는 정해진 변수이름은 아님
        self.get_logger().info(f'incomming data : {request.data}')

        sErrorMsg='Something Wrong...'
        bSuccess=False

        if request.data:
            # self.blink_led(21,3)
            self.publisher()
            sErrorMsg=''
            bSuccess=True

        response.message = sErrorMsg
        response.success = bSuccess
        return response
    
    def publisher(self):
        msg=String()
        msg.data='LED_BLINK'
        self.pub.publish(msg)
    
    def blink_led(pin, times):
        for _ in range(times):
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.5)



def main(args=None):
    rclpy.init(args=args)
    node = RobotGPIOHandling()

    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        # node.start_searching()
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        GPIO.cleanup()
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()


if __name__ == '__main__':
    main()
