import apriltag
import cv2 as cv
import numpy as np
import threading
import sys
import Adafruit_PCA9685
import time

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

cap = cv.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
at_detector =  apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
index = 1

lastError_x = 0
lastError_y = 0
w_center = 320
h_center = 240
x_dutyCircle = 435
y_dutyCircle = 220
para_x = [lastError_x, x_dutyCircle, w_center]
para_y = [lastError_y, y_dutyCircle, h_center]

pwm.set_pwm(9, 0, 435)
time.sleep(0.1)
pwm.set_pwm(8, 0, 220)


def set_pwm(center_param, list2):
    err = list2[2] - center_param
    print('err = ' + str(err))
    pwm_v = err * 0.04 + 0.001 * (err - list2[0])
    list2[0] = err
    list2[1] += pwm_v
    # if list2[1] > 13:
    #     list2[1] = 12
    # if list2[1] < 2.5:
    #     list2[1] = 3


while cap.isOpened():
    ret, frame = cap.read()
    # frame = cv.flip(frame, 1, dst=None)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)
    key = cv.waitKey(45)
    if key & 0x00FF == 27:
        break
    elif key == ord('s'):
        cv.imwrite('D:/imageTemp/' + str(index) + '.jpg', frame)
        index += 1
    for tag in tags:
        # cv.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)
        # cv.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)
        # cv.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)
        # cv.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)
        # cv.circle(frame, tuple(tag.center.astype(int)), 4, (0, 255, 0), 2)
        cx = tag.center[0].astype(int)
        cy = tag.center[1].astype(int)
        print(cx,cy)

        threads = []
        t1 = threading.Thread(target=set_pwm, args=(tag.center[0], para_x))
        threads.append(t1)
        t2 = threading.Thread(target=set_pwm, args=(tag.center[1], para_y))
        threads.append(t2)

        for t in threads:
            t.setDaemon(True)
            t.start()
            t.join()

        print('X_Duty->' + str(para_x[1]))
        print('Y_Duty->' + str(para_y[1]))
        pwm.set_pwm(8, 0, para_y[1].astype(int))
        time.sleep(0.1)
        # pwm.set_pwm(9, 0, 950 - para_x[1].astype(int))

    cv.imshow('capture', frame)
cap.release()
cv.destroyWindow()
