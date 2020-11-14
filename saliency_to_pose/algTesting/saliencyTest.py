# Author: Trevor Sherrard
# Since: Novemeber, 10, 2020

import cv2
import glob
import numpy as np
import math
import operator

def subLists(listA, listB):
    diffList = []
    for i in range(len(listA)):
        diffList.append(listA[i] - listB[i])

    return diffList

def countNumOverThresh(diffList, threshVal):
    count = 0
    for elem in diffList:
        if elem > 0 and elem > threshVal:
            count += 1

    return count

def euclidDist(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

def findMinimizationDir(imgCenterPt, momentCenterPts):
    threshVal = 40
    # compute initial distances
    initDist = []
    for elem in momentCenterPts:
        dist = euclidDist(elem, imgCenterPt)
        initDist.append(dist)

    # compute distance from new location
    newCenterPtUp = (imgCenterPt[0], imgCenterPt[1] - 50)
    newCenterPtDown = (imgCenterPt[0], imgCenterPt[1] + 50)
    newCenterPtLeft = (imgCenterPt[0] - 50, imgCenterPt[1])
    newCenterPtRight = (imgCenterPt[0] + 50, imgCenterPt[1])

    newDistUp = []
    newDistDown = []
    newDistLeft = []
    newDistRight = []
    for elem in momentCenterPts:
        distUp = euclidDist(elem, newCenterPtUp)
        distDown = euclidDist(elem, newCenterPtDown)
        distLeft = euclidDist(elem, newCenterPtLeft)
        distRight = euclidDist(elem, newCenterPtRight)

        newDistUp.append(distUp)
        newDistDown.append(distDown)
        newDistLeft.append(distLeft)
        newDistRight.append(distRight)

    subListUp = subLists(initDist, newDistUp)
    subListDown = subLists(initDist, newDistDown)
    subListLeft = subLists(initDist, newDistLeft)
    subListRight = subLists(initDist, newDistRight)

    countUp = countNumOverThresh(subListUp, threshVal)
    countDown = countNumOverThresh(subListDown, threshVal)
    countLeft = countNumOverThresh(subListLeft, threshVal)
    countRight = countNumOverThresh(subListRight, threshVal)

    dictToSort = {"Up":countUp, "Down":countDown, "Left":countLeft, "Right":countRight}
    if(dictToSort["Up"] == dictToSort["Down"] and dictToSort["Left"] == dictToSort["Right"]):
        print("done")
    elif(dictToSort["Up"] == dictToSort["Down"] and dictToSort["Left"] != dictToSort["Right"]):
        rightVal = dictToSort["Right"]
        leftVal = dictToSort["Left"]

        if(rightVal > leftVal):
            print("go Right")
        elif(rightVal < leftVal):
            print("go Left")
    elif(dictToSort["Up"] != dictToSort["Down"] and dictToSort["Left"] == dictToSort["Right"]):
        upVal = dictToSort["Up"]
        downVal = dictToSort["Down"]

        if(upVal > downVal):
            print("go Up!")
        elif(upVal < downVal):
            print("go Down!")

    else:
        maxVal = max(dictToSort.iteritems(), key=operator.itemgetter(1))[0]
        print("go " + maxVal + "!")
def computeSaliencyMap(img):
    
    # define constants
    maxVal = 255
    threshVal = 100
    minArea = 600

    # compute image center
    imgCY = np.size(img, 0)/2
    imgCX = np.size(img, 1)/2
    imgCenterPt = (imgCX, imgCY) 

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
        momentCenterPts = []
        for cont in filteredContours:

            # find COF of single contour
            M = cv2.moments(cont)
	    centerX = int(M["m10"] / M["m00"])
	    centerY = int(M["m01"] / M["m00"])
            MomentCenterPt = (centerX, centerY)

            # draw circle on COM, draw line between image center and COM
            cv2.circle(img, (centerX, centerY), 7, (255, 0, 0), -1)
            cv2.line(img, (centerX, centerY), (imgCX, imgCY), (0, 0, 0), thickness=2)

            # add COM point to list
            momentCenterPts.append(MomentCenterPt)

        # draw marker on image center
        cv2.circle(img, (imgCX, imgCY), 7, (255, 255, 255), -1)

        # find cardinal direction that minimizes distances between 
        # saliency COM and center of image
        findMinimizationDir(imgCenterPt, momentCenterPts)

        return img

fileList = glob.glob("/home/sherrardtr/catkin_ws/src/RBE526-Human-Robot-Interaction/saliency_to_pose/algTesting/images/*")

for img in fileList:
    cap = cv2.VideoCapture(0)
    while(cap.isOpened()):
        ret, frame = cap.read()
        
        saliencyMap = computeSaliencyMap(frame)
        cv2.imshow("test", saliencyMap)
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break

