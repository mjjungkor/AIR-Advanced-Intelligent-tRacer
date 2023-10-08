# Service 를 이용하여 영상처리 결과 확인 및 Timer를 이용한 로봇 이동
# 제자리에서 회전 하다가 타킷이 발견되면 이동

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
import math
from math import sqrt
import time

from mj_interfaces.srv import TargetCommand

# TargetCommand Service Interface
# int32 seq_id
# float32 info_x
# float32 info_y
# float32 info_theta
# float32[] info
# ---
# string info_result

from rclpy.executors import MultiThreadedExecutor

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class RobotHandling(Node):
    def __init__(self):
        super().__init__('robot_handling')
        self.qos_profile = QoSProfile(depth = 10)
        self.create_subscription(Odometry, 
                                 'odom', 
                                 self.sub_odom_callback, 10, callback_group=ReentrantCallbackGroup())
        self.create_service(TargetCommand,
                            'target_command', 
                            self.get_target_value,callback_group=ReentrantCallbackGroup()) 
        self.pub=self.create_publisher(Twist, 
                                       'cmd_vel', 10, callback_group=ReentrantCallbackGroup())
        self.create_timer(0.1,self.update)
        
        self.target=Twist() # for rotate(비전에 나온 목표좌표)
        self.origin=Twist() # for move(로봇의 초기좌표)
        
        self.bTargetRecog=False # 영상처리 결과 수신
        self.isMoving=False # 타켓으로 이동

        self.start_searching()
        self.target_distance=0.0

    # service(server) - 영상처리 결과
    def get_target_value(self, request, response):#request,response는 정해진 변수이름은 아님
        # 탐색 정지
        self.stop_searching()

        # self.target.linear.x=request.info_x
        # self.target.linear.y=request.info_y
        # self.target.angular.z=request.info_theta
        
        self.target.linear.x=request.info[0]
        self.target.linear.y=request.info[1]
        self.target.angular.z=request.info[2]
        # self.get_logger().info(f'Receive Servie({request.seq_id}) : ({self.target.linear.x},{self.target.linear.y},{self.target.angular.z})')
        
        self.target_distance=request.info[0]
        self.bTargetRecog=True
        
        response.info_result=f'Succeed({request.seq_id})'
        return response
    
    # 1초 주기의 타이머를 사용해도 내부에 소요시간이 오래 걸리는 실행문이 있으면 타이머 동작이 블록 됨
    def update(self): # 이미지처리 1번 결과(계산된 목표 이동 거리)
        if self.bTargetRecog:
            self.bTargetRecog=False
            self.origin.linear.x=self.robot_x
            self.origin.linear.y=self.robot_y
            self.move(self.target_distance)
            print('4.stop move')
        else:
            self.start_searching() 

    def stop_searching(self):
        print('1.stop searching')
        msg=Twist()
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def start_searching(self):
        if self.isMoving: return

        print('0.searching target')
        msg=Twist()
        msg.angular.z = 1.0
        self.pub.publish(msg)
    
    # 터틀봇 위치 및 방향 확인
    def sub_odom_callback(self, data):
        # print('odom')
        self.robot_x=data.pose.pose.position.x
        self.robot_y=data.pose.pose.position.y
        self.robot_theta = euler_from_quaternion(data.pose.pose.orientation)[2]
        # self.get_logger().info(f'robot(x,y,theta) : {self.robot_x}, {self.robot_y}, {self.robot_theta}')
        
    def euclidean_distance(self): # 현재 터틀 위치 - 초기 터틀 위치
        return sqrt(pow((self.robot_x - self.origin.linear.x), 2) + pow((self.robot_y - self.origin.linear.y), 2))
    
    # 로봇 이동 토픽/액션 발행
    def move(self, target_distance):
        print('2.start move')
        msg=Twist()

        msg.linear.x=0.05 # 이동속도지정
        print(f'start pos={self.origin.linear.x}, {self.origin.linear.y}')
        print(f'cmd dis={target_distance}')

        while rclpy.ok():
            self.pub.publish(msg)

            if self.euclidean_distance() >= target_distance: # 로봇이 지정된 이동 거리 이상이면 루프 탈출
                print('meet target_distance')
                break
        
        msg.linear.x=0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotHandling()

    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        # node.start_searching()
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        msg=Twist()
        msg.linear.x=0.0
        msg.linear.y=0.0
        msg.angular.z=0.0

        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()


if __name__ == '__main__':
    main()
