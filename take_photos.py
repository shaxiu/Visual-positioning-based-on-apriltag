#take photos
import cv2
camera=cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
i=0
while 1:
    (grabbed,img)=camera.read()
    cv2.imshow('img',img)
    if cv2.waitKey(1)&0xFF==ord('j'):
        i+=1
        u=str(i)
        firename=str('/home/pi/Documents/CODE/pic/'+u+'.jpg')
        cv2.imwrite(firename,img)
        print('写入',firename)
    c=cv2.waitKey(10)
    if c==27:
        break