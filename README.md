# Line-Following-Robot

The line follower follows a 1.5cm black line on a white background. It uses 8 infrared sensors to detect the light and then follow it using PID logic. The IR sensor works by emmitting infrared rays and then varying the resistance based on the intensity of light recieved. Based on this the arduino detects where the line exactly is relative to it. 

PCB design is attached in file X1.pdf

Components Used:

- Arduino Nano x 1 
- TB6612FNG x 1
- Electronic Speed Controller (ESC) x 1 
- Elecronic Ducted Fan (EDF) x 1
- Pololu 12v micro metal gear motor x 2
- IR sensors x 8
- JSumo IR Switch x 1
- Turnigy 460 mah 2s 20C - 40C LiPo battery x 1

The file LF_Slow.ino contains the slower code in which the PWM of the motors and the EDF is less while the file LF_Fast.ino contains the faster code in which the PWM of the motors and the EDF is greater.


Test Run - https://www.youtube.com/shorts/rUErMrPgKn0

Final Run - https://www.youtube.com/shorts/a4Mq1eNVcf0
