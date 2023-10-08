import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time
import RPi.GPIO as GPIO

class Sim_sub(Node):
    def __init__(self):
        super().__init__('robot_ext_devices')
        #DDS에는 string type이 없어 'from std_msgs.msg import String' 진행
        #10은 메시지를 받을 buffer의 크기
        self.create_subscription(String, 'robot_LED_control', self.subcallback, 10)        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(21, GPIO.OUT)

    def subcallback(self, msg):
        if msg.data == 'LED_BLINK':
            self.blink_led(21,3)
        self.get_logger().info(f'Received Message : {msg.data}')#시간/출처(노드이름) 정보 자동 추가
        
    def blink_led(self, pin, times):
        for _ in range(times):
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.5)

def main():
    rclpy.init()
    node=Sim_sub()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('KeyboardInterrupt Error')
    finally:
        GPIO.cleanup()
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__=='__main__':
    main()