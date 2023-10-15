[Laptop Computer]

1) Image Processing
 - Turtlebot에서 발행되는 CompressedImage자료형의 camera/image/compressed Topic 구독
 - Haar Cascade Algorithm을 이용하여 목표 이미지('STOP' Sign) 등록 및 처리
 - 카메라와 목표물까지 거리는 삼각법을 이용하여 계산
  D = (W * F) / P
  P = Sqrt((F_W / 2 - T_X)^2 + (F_H / 2 - T_Y)^2)
  D[m]: 객체까지의 거리
  W[m]: 객체의 실제 폭 또는 높이
  F[mm]: 카메라의 초점 거리
  P[pixel]: 이미지 중심에서 객체의 중심까지의 픽셀 거리 계산
  F_W[pixel]: 이미지 너비(FOV WIDTH)
  F_H[pixel]: 이미지 높이(FOV HEIGHT)
  T_X[pixel]: 객체 중심의 X좌표
  T_Y[pixel]: 객체 중심의 Y좌표
 - user define service interface(TargetCommand자료형의 target_command) Service를 통해 처리 결과를 Turtlebot을 제어하는 node로 전송

2) Robot Operating
 - Turtlebot의 이동/정지 Topic 발행 제어는 타이머 이용
 - 표식을 찾기 위해 제자리에서 회전 동작 수행
 - 표식 탐지 신호(TargetCommand자료형의 target_command)가 이미지 처리 node로부터 수신되면 회전 동작을 정지하고
 - 이미지 처리에서 계산된 목표 이동거리만큼 Turtlebot 이동하기 위해 현재 Turtlebot 이동 거리를 계산하면서 Twist 자료형의 cmd_vel Topic 발행
