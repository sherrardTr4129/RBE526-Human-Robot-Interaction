# Author: Trevor Sherrard
# Since: Novemeber, 10, 2020

import cv2
import glob
import numpy as np
import math

def euclidDist(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

def computeSaliencyMap(img):
    
    # define constants
    maxVal = 255
    threshVal = 100
    minArea = 300

    # compute image center
    imgCY = np.size(img, 0)/2
    imgCX = np.size(img, 1)/2

    # compute saliency image
    saliencyObj = cv2.saliency.StaticSaliencyFineGrained_create()
    (success, saliencyMap) = saliencyObj.computeSaliency(img) 
    if(success):
        # scale image to (0, 255)
        saliencyMap = (saliencyMap * 255).astype("uint8")

        #blur image
        blur = cv2.GaussianBlur(saliencyMap, (5, 5), 0)

        # threshold image
        ret, thresh = cv2.threshold(blur, threshVal, maxVal, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # filter detected contours
        filteredContours = []
        for cont in contours:
            area = cv2.contourArea(cont)
            if(area > minArea):
                filteredContours.append(cont)
        
        # draw filtered contours
        cv2.drawContours(img, filteredContours, -1, (0, 255, 0), -1)
        
        # find center of all filter contours
        for cont in filteredContours:
            M = cv2.moments(cont)
	    centerX = int(M["m10"] / M["m00"])
	    centerY = int(M["m01"] / M["m00"])
            cv2.circle(img, (centerX, centerY), 7, (255, 0, 0), -1)
            cv2.line(img, (centerX, centerY), (imgCX, imgCY), (0, 0, 0), thickness=2)

        # draw marker on image center
        cv2.circle(img, (imgCX, imgCY), 7, (255, 255, 255), -1)
        
        return img

fileList = glob.glob("/home/sherrardtr/catkin_ws/src/RBE526-Human-Robot-Interaction/saliency_to_pose/algTesting/images/*")

for img in fileList:
    frame = cv2.imread(img)
    saliencyMap = computeSaliencyMap(frame)
    cv2.imshow("test", saliencyMap)
    cv2.waitKey(0)

