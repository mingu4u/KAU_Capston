from types import GeneratorType
import cv2
import numpy as np
from os.path import exists

# from detector import ENUM_PURPLE

ENUM_PURPLE = 0
ENUM_GREEN = 1
ENUM_RED = 2
ENUM_QR = 3

COLOR_TEXT = {
    ENUM_PURPLE : "Purple",
    ENUM_GREEN : "Green",
    ENUM_RED : "Red"
}

def qrDetect(frame):

    # initialize the OpenCV QRCode detector
    detector = cv2.QRCodeDetector()
    data, vertices_array, _ = detector.detectAndDecode(frame)
    # check if there is a QRCode in the image
    if vertices_array is not None:
        vertices_array = vertices_array.astype(int)
        # if data:
        #     # print("QR Code detected, data:", data)
        #     cv2.rectangle(frame, vertices_array[:,0].reshape(-1), vertices_array[:,2].reshape(-1),(0,255,0),3)
        #     # print(data)
        return data
    else:
        return []

def calContour(color, frame):
    colorExclude = [0,1,2]
    colorExclude.pop(color)
    # print(f"Blue : {frame[-1,-1,0]} \t Green : {frame[-1,-1,1]} \t Red : {frame[-1,-1,2]} ")

    if(color == ENUM_RED):
        detector = np.logical_and(np.logical_and(frame[:,:, colorExclude[0]] < frame[:,:,color], frame[:,:,colorExclude[1]] < frame[:,:,color]), frame[:,:,color] > 123)
        colorMask = np.logical_and(frame[:,:, colorExclude[0]] < 100, frame[:,:,colorExclude[1]] < 100)
        maskedImage = np.array(np.logical_and(detector,colorMask)*255, dtype=np.uint8)
    elif(color == ENUM_PURPLE):
        detector = np.logical_and(np.logical_and(frame[:,:, colorExclude[0]] < frame[:,:,color], frame[:,:,colorExclude[1]] < frame[:,:,color]), frame[:,:,color] > 120)
        colorMask = np.logical_and(frame[:,:, colorExclude[0]] < 80, frame[:,:,colorExclude[1]] < 80)
        maskedImage = np.array(np.logical_and(detector, colorMask)*255, dtype=np.uint8)
    elif(color == ENUM_GREEN):
        detector = np.logical_and(np.logical_and(frame[:,:, colorExclude[0]] < frame[:,:,color], frame[:,:,colorExclude[1]] < frame[:,:,color]), frame[:,:,color] > 80)
        colorMask = np.logical_and(frame[:,:, colorExclude[0]] < 70, frame[:,:,colorExclude[1]] < 70)
        maskedImage = np.array(np.logical_and(detector,colorMask)*255, dtype=np.uint8)

    contours, hierarchy = cv2.findContours(maskedImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours



def getRect(frame, colors, areaSize, WIDTH, HEIGHT):
    frame_copy = frame.copy()

    contourDict = {}

    for color in colors:
        contours = calContour(color, frame)
        pos = []

        saveFrame = frame.copy()
        for contour in contours:

            (x,y,w,h) = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)

            if(area < areaSize or 1.5 < h/w or h/w < 0.7 ):
                # print(area)
                continue

            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.rectangle(saveFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)


            cv2.putText(frame_copy, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
            cv2.putText(saveFrame, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

            cv2.imwrite(f"./{COLOR_TEXT[color]}.png", saveFrame)
            pos.append(np.array(((x,y), (x+w,y), (x,y+h), (x+w,y+h))))
        
        # if(not exists(f"./{COLOR_TEXT[color]}.png")):

        contourDict[color] = pos
        # print(contourDict)

    data = qrDetect(frame_copy)
    contourDict[ENUM_QR] = data

    cv2.imshow("original", frame_copy)
    return contourDict

def getCircle(frame, colors, areaSize, WIDTH, HEIGHT):
    contourDict = {}

    if(len(colors) == 0):
        contourDict = {ENUM_PURPLE : None, ENUM_GREEN : None, ENUM_RED : None}
    else:
        for color in [ENUM_PURPLE, ENUM_GREEN, ENUM_RED]:
            contourDict[color] = None

        for color in colors:
            circle = calContourHSV(color, frame)
            if(circle is None):
                contourDict[color] = None
                continue
            pos = []
            saveFrame = frame.copy()

            (x,y,r,c) = map(int, circle[:-1])
            rect = list(map(int,circle[-1]))
            
            
            cv2.circle(frame, (int(circle[0]), int(circle[1])), int(circle[2]), (0, 255, 100), 3)
            cv2.circle(saveFrame, (int(circle[0]), int(circle[1])), int(circle[2]), (0, 255, 100), 3)
            

            cv2.putText(frame, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
            cv2.putText(saveFrame, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
            
            # cv2.putText(frame, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
            # cv2.putText(saveFrame, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
            pos.append(np.array((x,y)))
            contourDict[color] = pos
    cv2.imshow("original", frame)
    return contourDict




def calContourHSV(color, frame):
    colorExclude = [0,1,2]
    colorExclude.pop(color)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    if(color == ENUM_RED):
        # lower_red1 = np.array([0//2, 255//3, 50])
        # upper_red1 = np.array([15//2, 255, 200])

        # lower_red2 = np.array([345//2, 255//3, 50])
        # upper_red2 = np.array([360//2, 255, 200])
        
        lower_red1 = np.array([0//2, 255, 200])
        upper_red1 = np.array([0//2, 255, 200])

        lower_red2 = np.array([359//2, 255//3, 50])
        upper_red2 = np.array([360//2, 255, 200])

        rowMask = cv2.inRange(hsv, lower_red1, upper_red1)
        upperMask = cv2.inRange(hsv, lower_red2, upper_red2)

        maskedImage = cv2.bitwise_or(rowMask, upperMask)
        # maskedImage = img
    
    elif(color == ENUM_PURPLE):
        lower_blue = np.array([145, 60, 60])
        upper_blue = np.array([170, 165, 150])
        maskedImage = cv2.inRange(hsv, lower_blue, upper_blue)
        # cv2.imshow("test1", maskedImage)
        
    elif(color == ENUM_GREEN):
        lower_blue = np.array([30,80,100])
        upper_blue = np.array([65, 140, 205])
        maskedImage = cv2.inRange(hsv, lower_blue, upper_blue)

        cv2.imshow("test", maskedImage)
    new = frame.copy()
    contours, _ = cv2.findContours(maskedImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if(len(contours) != 0):  
        c = max(contours, key=cv2.contourArea)
        if(cv2.contourArea(c) > 40):
            (x,y),r = cv2.minEnclosingCircle(c)
            rect = cv2.boundingRect( c )

            return (x,y,r, cv2.contourArea(c), rect)
        else:
            return None
    else:
        return None
