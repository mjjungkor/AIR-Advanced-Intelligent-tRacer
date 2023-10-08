# Service 를 이용한 영상처리 이미지 결과 전달

# 카메라 하드웨어 재원
# https://www.raspberrypi.com/documentation/accessories/camera.html

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile # Quality of Service
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
import numpy as np
import math

from mj_interfaces.srv import TargetCommand
import os, time


class ImageHandling(Node):
    def __init__(self):
        super().__init__('image_handling')
        self.qos_profile = QoSProfile(depth = 10)
        self.create_subscription(CompressedImage, 'camera/image/compressed', self.sub_img_callback, 10,callback_group=ReentrantCallbackGroup())
        # self.publisher = self.create_publisher(Twist, 'command_pose', 10,callback_group=ReentrantCallbackGroup())
        self.cb = CvBridge()
        self.register_target_img()
        self.focal_length = 3.6  # 초점 거리[mm]
        # ~/turtlebot3_ws/src/raspicam2_node/cfg$/params.yaml
        fov_width=320
        fov_height=240
        self.image_center = (int(fov_width/2), int(fov_height/2))  # 이미지 중심 좌표
        self.const_match_x=1.0 # 실험을 통해 등록된 이미지와 카메라를 사용해서 매치되었을때 거리 고정 상수

        self.client=self.create_client(TargetCommand,'target_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.msg=TargetCommand.Request()
        self.seq_id=0
        # self.timer=self.create_timer(1,self.timer_callback) # Service Call Test Code

    # def timer_callback(self): # Service Call Test Code
    #     cmd_pose=Twist()
    #     cmd_pose.linear.x=1.0
    #     cmd_pose.linear.y=0.1
    #     cmd_pose.angular.z=3.0
    #     self.call_service(cmd_pose)
    
    def call_service(self, data):
        # docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#filed-types
        self.seq_id+=1
        self.msg.seq_id=self.seq_id
        self.msg.info=[data.linear.x,data.linear.y,data.angular.z]
        
        future = self.client.call_async(self.msg) # 반드시 call_async
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        if future.result() is not None:
            self.get_logger().info(f'Received message: {future.result().info_result}')
        else:
            self.get_logger().error('Service call failed')


    # 목표 이미지 등록
    def register_target_img(self): # for Haar Cascades
        self.star_cascade = cv2.CascadeClassifier('/home/mjjung/colcon_ws/src/air/data/stop_sign_classifier_2.xml')
        print('loaded haarcascade')

    def sub_img_callback(self, data):
        try:
            frame = self.cb.compressed_imgmsg_to_cv2(data) #ros이미지 포맷을 cv2이미지 포맷으로 변경
            cmd_pose=Twist()
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            binary=cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 101,7)
            targets = self.star_cascade.detectMultiScale(binary, scaleFactor = 1.2,minNeighbors = 5)

            # if len(targets) <1: return

            for (x,y,w,h) in targets:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
                delta_x = int(x+w/2) - self.image_center[0]
                delta_y = int(y+h/2) - self.image_center[1]
                # 카메라 초점 거리를 이용하여 객체까지의 거리 계산(ChatGPT)
                distance_to_object = (self.focal_length * 2) / (delta_x**2 + delta_y**2)
                cv2.putText(frame,'distance:%.3f'%distance_to_object,(10,10), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),1)
                
                # print(distance_to_object)
                # 로봇과 객체 간의 거리 계산
                # robot_to_object_distance = np.sqrt((robot_position[0] - obj_cen[0])**2 + (robot_position[1] - obj_cen[1])**2)

                # 1.계산된 이동 거리
                cmd_pose.linear.x=distance_to_object

                # 2.좌표
                # cmd_pose.linear.x=obj_cen[0]
                # cmd_pose.linear.y=obj_cen[1]

                self.call_service(cmd_pose) # 이미지 처리 결과 서비스콜

            cv2.waitKey(1)
            cv2.imshow('acquire image from camera', frame) # 원본 이미지
            cv2.imshow('processing binary', binary) # 처리된 이미지
            
        except CvBridgeError as e:
            self.get_logger().info(e)



def main(args=None):
    rclpy.init(args=args)
    node = ImageHandling()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
