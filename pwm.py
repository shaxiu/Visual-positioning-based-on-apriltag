import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

# 设置最大最小脉冲长度
servo_up_min = 160
servo_up_max = 260
servo_down_min = 375
servo_down_max = 495
servo_up_mid = 220  # 越大越仰
servo_down_mid = 435  # 越大越往逆时针走
# 工作频率为50
pwm.set_pwm_freq(50)
print('Moving servo on, press Ctrl-C to quit...')
pwm.set_pwm(8, 0, 220)
# time.sleep(1)
# pwm.set_pwm(9, 0, servo_down_max)
# while True:
#     # pwm.set_pwm(8, 0, servo_up_min)
#     # time.sleep(1)
#     # pwm.set_pwm(8, 0, servo_up_max)
#     # time.sleep(1)
#     # pwm.set_pwm(9, 0, servo_down_min)
#     # time.sleep(1)
#     # pwm.set_pwm(9, 0, servo_down_max)
#     # time.sleep(1)
