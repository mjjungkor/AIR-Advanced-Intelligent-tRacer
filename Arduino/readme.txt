[Arduino Uno Device Code]

1) INPUT
 - 입력 센서로부터 신호를 받아 신호가 토글(OFF → ON)된 경우
 - 시리얼 통신을 통해 'ON' 문자열을 Raspberry Pi 4b로 전송
2) OUTPUT
 - 일종의 Cross Check Mechanism으로 송신한 신호를 Raspberry Pi 4b에서 제대로 수신했는지에 대한 물리적인 확인을 위해
 - 시리얼 포트로 'OK'문자열을 수신하면 LED를 3회 점멸 시킴
