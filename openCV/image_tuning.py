from detector import getRect
import detector
import cv2
import djitellopy
cap = cv2.VideoCapture(0)
width = int(cap.get(3)) # 가로 길이 가져오기 
height = int(cap.get(4)) # 세로 길이 가져오기
WIDTH = 960
HEIGHT = 720
fps = 20
if __name__ == '__main__':

    i = 0
    fcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    out = cv2.VideoWriter('webcam.avi', fcc, fps, (width, height), isColor=False)
    print(out.isOpened())
    while True:
        i+=1
        ret, frame = cap.read()
        center = (width//2,height//2)
        if ret :
            cv2.circle(frame, center, 10, (0,255,0), 3)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.imshow("Tuning", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if(i%15==0):
                print(frame.shape)
                # print(f"Blue : {frame[height//2,width//2,0]} \t Green : {frame[height//2,width//2,1]} \t Red : {frame[height//2,width//2,2]} ")
                print(f"Blue : {hsv[height//2,width//2,0]} \t Green : {hsv[height//2,width//2,1]} \t Red : {hsv[height//2,width//2,2]} ")