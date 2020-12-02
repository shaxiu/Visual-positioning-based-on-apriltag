# coding=utf-8
import apriltag     
import cv2
import numpy as np
import sys
import math
def RotateByZ(Cx, Cy, thetaZ):
    rz = thetaZ*math.pi/180.0
    outX = math.cos(rz)*Cx - math.sin(rz)*Cy
    outY = math.sin(rz)*Cx + math.cos(rz)*Cy
    return outX, outY
def RotateByY(Cx, Cz, thetaY):
    ry = thetaY*math.pi/180.0
    outZ = math.cos(ry)*Cz - math.sin(ry)*Cx
    outX = math.sin(ry)*Cz + math.cos(ry)*Cx
    return outX, outZ
def RotateByX(Cy, Cz, thetaX):
    rx = thetaX*math.pi/180.0
    outY = math.cos(rx)*Cy - math.sin(rx)*Cz
    outZ = math.sin(rx)*Cy + math.cos(rx)*Cz
    return outY, outZ
   

cameraParams_Intrinsic = [591,591,321,245]  # camera_fx, camera_fy, camera_cx, camera_cy 
camera_matrix = np.array(([591.280996, 0, 321.754492],
                         [0, 591.853914, 245.866165],
                         [0, 0, 1.0]), dtype=np.double)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 5)
tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))  # Build a detector for apriltag
while( cap.isOpened() ):
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # The image must be a grayscale image of type numpy.uint8
    key = cv2.waitKey(45)
    if key & 0x00FF == 27:
        break
    tags = tag_detector.detect( gray)
    for tag in tags:
        cv2.circle(img, tuple(tag.corners[0].astype(int)), 4,(0,0,255), 2) # left-top
        cv2.circle(img, tuple(tag.corners[1].astype(int)), 4,(0,0,255), 2) # right-top
        cv2.circle(img, tuple(tag.corners[2].astype(int)), 4,(0,0,255), 2) # right-bottom
        cv2.circle(img, tuple(tag.corners[3].astype(int)), 4,(0,0,255), 2) # left-bottom
        cv2.circle(img, tuple(tag.center.astype(int)), 4,(0,0,255), 2) #center
        #cv2.circle(img, tuple(tag.center.a, 4,(0,0,255), 2) # left-bottom         
        #print(tag.tag_id)   #output the tag_id
        # print(tag.center)
        '''object_3d_points = np.array(([-6.3, 6.3, 0],
                                    [6.3, 6.3, 0],
                                    [6.3, -6.3, 0],
                                    [-6.3,-6.3, 0]),
                                    dtype=np.double)'''    # Apriltag coordinates in the World coordinate system
        object_3d_points = np.array(([-7.3, 7.3, 0],
                                    [7.3, 7.3, 0],
                                    [7.3, -7.3, 0],
                                    [-7.3,-7.3, 0]),
                                    dtype=np.double)    # Apriltag coordinates in the World coordinate system

        object_2d_point = np.array((tag.corners[0].astype(int),
                                    tag.corners[1].astype(int),
                                    tag.corners[2].astype(int),
                                    tag.corners[3].astype(int)),
                                    dtype=np.double)    # Apriltag coordinates in the Image pixel system

        dist_coefs = np.array([0.09725213,-0.08208706,0.00204942,-0.00821362,-0.18558094], dtype=np.double)    # Distortion coefficient: k1, k2, p1, p2, k3

        found, rvec, tvec = cv2.solvePnP(object_3d_points, object_2d_point, camera_matrix, dist_coefs)  #rvec-旋转向量  tvec-平移向量
        rotM = cv2.Rodrigues(rvec)[0]   #旋转向量转换为旋转矩阵
        # print(rotM)
        # print('--------------------------')
        # print(cv2.Rodrigues(rvec))
        camera_postion = -np.matrix(rotM).T * np.matrix(tvec)
        #通过旋转矩阵计算欧拉角
        thetaZ = math.atan2(rotM[1, 0], rotM[0, 0])*180.0/math.pi
        thetaY = math.atan2(-1.0*rotM[2, 0], math.sqrt(rotM[2, 1]**2 + rotM[2, 2]**2))*180.0/math.pi
        thetaX = math.atan2(rotM[2, 1], rotM[2,2])*180.0/math.pi
    
        x = tvec[0]
        y = tvec[1]
        z = tvec[2]
        (x, y) = RotateByZ(x, y, -1.0*thetaZ)
        (x, z) = RotateByY(x, z, -1.0*thetaY)
        (y, z) = RotateByX(y, z, -1.0*thetaX)
        Cx = x*-1
        Cy = y*-1
        Cz = z*-1
    
        print("camera position:",Cx, Cy, Cz)       
        #print("camera rotation:", thetaX+180, thetaY, thetaZ)  
        #print("camera rotation:", thetaX)
        #print("camera position:",Cx)     
    # Extra points for debug the accuracy
       
    cv2.imshow('capture', img)
    
cap.release()
cv2.destroyAllWindows()
