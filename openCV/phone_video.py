from pickle import NONE
import cv2
import detector
import numpy as np
import time
cap = cv2.VideoCapture(0)

# width = int(cap.get(3)) # 가로 길이 가져오기 
# height = int(cap.get(4)) # 세로 길이 가져오기
width = 3200 # 가로 길이 가져오기 
height = 1440 # 세로 길이 가져오기
fps = 20
WIDTH = 3200
HEIGHT = 1440
#3200 1440
fcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
out = cv2.VideoWriter('webcam.avi', fcc, fps, (width, height), isColor=False)
print(out.isOpened())
while (True) :
    ret, frame = cap.read()
    if ret :
        ## 회색으로 opencv 화면 출
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # out.write(gray)
        # cv2.imshow('frame', gray)
        start = time.time()
        out.write(frame)
        cv2.imshow('frame', frame)
        recPos= detector.getCircle(frame,[detector.ENUM_PURPLE ,detector.ENUM_GREEN, detector.ENUM_RED],40, width, height)   
        # print(recPos[detector.ENUM_RED])
        # print(type(recPos[detector.ENUM_RED]))
        if recPos[detector.ENUM_PURPLE] and recPos[detector.ENUM_GREEN] != None:
            PURPLE_x = recPos[detector.ENUM_PURPLE][0][0]
            PURPLE_y = recPos[detector.ENUM_PURPLE][0][1]
            GREEN_x = recPos[detector.ENUM_GREEN][0][0]
            GREEN_y = recPos[detector.ENUM_GREEN][0][1]
            
        # RED_x,RED_y = recPos[detector.ENUM_RED][0],recPos[detector.ENUM_RED][1] 
        # GREEN_x,GREEN_y = recPos[detector.ENUM_GREEN][0],recPos[detector.ENUM_GREEN][1]
            dx = (GREEN_x-PURPLE_x)
            dy = (GREEN_y-PURPLE_y)
            angle_diff = np.degrees(np.arctan(dy / dx))
            
            angle_diff = format(angle_diff, ".5f")
            # ANGLE = ("각도 : %2f [Deg]" %angle_diff)
            cv2.putText(frame, str(angle_diff), (width//2,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.imshow("original", frame)
            dt = time.time() - start
            dt = format(dt, ".8f")
            print(dt , angle_diff)
            f = open("test6.txt", 'a')
            data = "%.8f          %.4f      \n" %(float(dt),float(angle_diff))
            f.write(data)
        if cv2.waitKey(1) & 0xFF == ord('q') : 
            f.close()
            break
    else :
        print("Fail to read frame!")
        
        break

cap.release()
out.release()
cv2.destroyAllWindows()