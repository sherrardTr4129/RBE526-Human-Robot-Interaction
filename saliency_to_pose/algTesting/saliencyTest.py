# Author: Trevor Sherrard
# Since: Novemeber, 10, 2020

import cv2
import glob

def computeSaliencyMap(img):
    saliencyObj = cv2.saliency.StaticSaliencyFineGrained_create()
    (success, saliencyMap) = saliencyObj.computeSaliency(img)

    if(success):
        return saliencyMap

fileList = glob.glob("/home/sherrardtr/catkin_ws/src/RBE526-Human-Robot-Interaction/saliency_to_pose/algTesting/images/*")

for img in fileList:
    frame = cv2.imread(img, 0)
    saliencyMap = computeSaliencyMap(frame)
    cv2.imshow("test", saliencyMap)
    cv2.waitKey(0)

