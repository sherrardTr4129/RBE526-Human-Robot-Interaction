#!/usr/bin/env python
# Author: Jialin Song
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander.conversions import pose_to_list
from math import pi
import math

# declare camera arm pose topics
camArmPoseTopic = "/currentCamArmPose"
camArmPoseGoalTopic = "/camArmPoseGoal"

# uncomment appropriate image topic
imageTopic = "/trina2_1/right_arm_cam/color/image_raw"
# imageTopic = "/trina2_1/left_arm_cam/color/image_raw"

# create global Pose message
global camArmPose, cokeArea, gripperArea, cokeCOM, gripperCOM, occluderArea, occluderCOM, state, BGR_IMAGE
camArmPose = Pose()
cokeArea = -1
gripperArea = -1
occluderArea = -1
cokeCOM = (-1, -1)
gripperCOM = (-1, -1)
occluderCOM = (-1, -1)
state = 0
BGR_IMAGE = np.zeros((720, 858, 3), dtype="uint8")

# set up Pose publisher for newly computed Pose
posePub = rospy.Publisher(camArmPoseGoalTopic, Pose, queue_size=10)
contourPub = rospy.Publisher('/raw_image_with_contour', Image, queue_size=10)


def procImage(img):
    """
    This function processes the recieved ROS image
    and calculates an updated pose for the end-effector
    """
    image = img
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv_img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    # gray_im = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    # camera info: 720 in height * 858 in width
    crop_hsv_img = hsv_img[0:585, 0:857]
    crop_img_rgb = img_rgb[0:585, 0:857]
    cx_can = 0.0
    cy_can = 0.0
    cx_gripper = 0.0
    cy_gripper = 0.0

    # coke can filter
    kernel = np.ones((7, 7))
    mask = cv2.inRange(hsv_img, (0, 15, 55), (13, 231, 103))
    mask = cv2.dilate(mask, kernel, iterations=3)
    mask = cv2.erode(mask, kernel, iterations=3)
    result = cv2.bitwise_and(img, img, mask=mask)
    gray = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
    edge = cv2.Canny(gray, 30, 200)
    contour, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    if len(contour) is not 0:
        cnt = contour[0]
        hull = cv2.convexHull(cnt, True)
        area = cv2.contourArea(hull, False)
        # rospy.loginfo("area of contour: " + str(area))
        M = cv2.moments(cnt)
        try:
            if M['m00'] is not 0.0:
                cx_can = int(float(M['m10']) / float(M['m00']))
                cy_can = int(float(M['m01']) / float(M['m00']))
            else:
                cx_can = int(M['m10'])
                cy_can = int(M['m01'])
            image = cv2.drawContours(img, hull, -1, (255, 0, 0), 2)
        except ZeroDivisionError:
            cx_can = -1
            cy_can = -1
        image = cv2.circle(image, (int(cx_can), int(cy_can)), 3, (255, 0, 0), 2)
    else:
        # rospy.loginfo("did not detect coke can")
        hull = [(0, 0)]
        area = -1
    cokeArea = area
    cokeCOM = (cx_can, cy_can)

    # blue box filter
    cx_occ = 0.0
    cy_occ = 0.0
    kernel2 = np.ones((1, 1))
    mask2 = cv2.inRange(hsv_img, (101, 64, 33), (160, 255, 225))
    mask2 = cv2.dilate(mask2, kernel2, iterations=1)
    mask2 = cv2.erode(mask2, kernel2, iterations=1)
    result2 = cv2.bitwise_and(img, img, mask=mask2)

    gray2 = cv2.cvtColor(result2, cv2.COLOR_RGB2GRAY)
    edge2 = cv2.Canny(gray2, 30, 200)
    contour2, _ = cv2.findContours(edge2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    if len(contour2) is not 0:
        cnt2 = contour2[0]
        occluder_area = cv2.contourArea(cnt2, False)
        M2 = cv2.moments(cnt2)
        try:
            if M2['m00'] is not 0.0:
                cx_occ = int(float(M2['m10']) / float(M2['m00']))
                cy_occ = int(float(M2['m01']) / float(M2['m00']))
            else:
                cx_occ = int(M2['m10'])
                cy_occ = int(M2['m01'])
            image = cv2.drawContours(image, contour2, -1, (0, 255, 0), 2)
        except ZeroDivisionError:
            cx_occ = -1
            cy_occ = -1
        image = cv2.circle(image, (int(cx_occ), int(cy_occ)), 3, (0, 255, 0), 2)
    else:
        # rospy.loginfo("did not detect occluder")
        occluder_area = -1
    occluderArea = occluder_area
    occluderCOM = (cx_occ, cy_occ)

    # left hand gripper
    mask3 = cv2.inRange(crop_hsv_img, (0, 0, 108), (1, 1, 255))
    result3 = cv2.bitwise_and(crop_img_rgb, crop_img_rgb, mask=mask3)
    gray3 = cv2.cvtColor(result3, cv2.COLOR_RGB2GRAY)
    edge3 = cv2.Canny(gray3, 30, 200)
    contour3, _ = cv2.findContours(edge3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    if len(contour3) is not 0:
        cnt3 = contour3[0]
        gripper_area = cv2.contourArea(cnt3, False)
        M3 = cv2.moments(cnt3)
        try:
            if M3['m00'] is not 0.0:
                cx_gripper = int(float(M3['m10']) / float(M3['m00']))
                cy_gripper = int(float(M3['m01']) / float(M3['m00']))
            else:
                cx_gripper = int(M3['m10'])
                cy_gripper = int(M3['m01'])
            image = cv2.drawContours(image, contour3, -1, (0, 0, 255), 2)
        except ZeroDivisionError:
            cx_gripper = -1
            cy_gripper = -1
        image = cv2.circle(image, (int(cx_gripper), int(cy_gripper)), 3, (0, 0, 255), 2)
    else:
        # rospy.loginfo("did not detect left hand gripper")
        gripper_area = -1
    gripperArea = gripper_area
    gripperCOM = (cx_gripper, cy_gripper)

    return image, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM


def all_close(goal, actual, tolerance):
    g = pose_to_list(goal)
    a = pose_to_list(actual)
    for ii in range(len(g)):
        if (abs(a[ii] - g[ii]) > tolerance) and \
                (abs(abs(a[ii] - g[ii]) - 2 * pi) > tolerance):
            return False
    return True


def imageCallback(msg):
    """
    This function serves as the ROS subscription callback. It converts
    the raw image data to an OpenCV compatable format.

    params:
        msg -> The recieved ROS image message.
    returns:
        None
    """
    global BGR_IMAGE
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    BGR_IMAGE = orig
    # rospy.loginfo(str(orig.shape))
    # rospy.loginfo(str(orig.dtype))
    # process image
    img,_,_,_,_,_,_ = procImage(orig)
    im_message = bridge.cv2_to_imgmsg(img, 'bgr8')
    contourPub.publish(im_message)


def updatePose(msg):
    """
    this function updates the global pose variable with
    newly recieved values

    params:
        msg -> The recieved Pose message
    returns:
        None
    """
    # reference globals so they can be used here
    global camArmPose

    # unpack message data
    camArmPose = msg


def initial_pose():
    ini_pose = Pose()
    ini_pose.position.x = .25  # -.1 #-.25
    ini_pose.position.y = -.35  # -.18
    ini_pose.position.z = .85  # 1.4 for occlusionPick
    arr = quaternion_from_euler(1.757 + 0.524, 0.4, 2.6)
    ini_pose.orientation.x = arr[0]
    ini_pose.orientation.y = arr[1]
    ini_pose.orientation.z = arr[2]
    ini_pose.orientation.w = arr[3]
    posePub.publish(ini_pose)


def setup_state():
    global cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM, BGR_IMAGE
    rospy.loginfo("setup state start...")
    rospy.sleep(1)
    initial_pose()
    rospy.sleep(1)
    # if cokeArea is -1 or gripperArea is -1:
    #     rospy.loginfo("Unable to detect goal or gripper... Please restart...")
    # else:
    #     if cokeCOM[0] == cokeCOM[1] == -1 or gripperCOM[0] == gripperCOM[1] == -1:
    #         pass
    #     else:
    #         while not checkCOM(cokeCOM, gripperCOM):
    #             adjust_view()

    rospy.loginfo("setup state finished...")


def try_view(x,y,z,r,p,ya):
    # TODO: required changing, updating global value
    global camArmPose, occluderArea, cokeArea
    pose = Pose()
    pose.position.x = camArmPose.position.x + x
    pose.position.y = camArmPose.position.y + y
    pose.position.z = camArmPose.position.z + z
    arrE = euler_from_quaternion([camArmPose.orientation.x, camArmPose.orientation.y,
                                  camArmPose.orientation.z, camArmPose.orientation.w])
    r = arrE[0] + r
    p = arrE[1] + p
    y = arrE[2] + ya
    arrQ = quaternion_from_euler(r, p, y)
    pose.orientation.x = arrQ[0]
    pose.orientation.y = arrQ[1]
    pose.orientation.z = arrQ[2]
    pose.orientation.w = arrQ[3]
    if not all_close(pose, camArmPose, .005):
        posePub.publish(pose)
    else:
        rospy.loginfo("Value too small for moving")
    rospy.sleep(1)
    return cokeArea, occluderArea


def adjust_view():
    global camArmPose
    pose = Pose()
    pose.position.x = camArmPose.position.x + 0.02
    pose.position.y = camArmPose.position.y
    pose.position.z = camArmPose.position.z
    arrE = euler_from_quaternion([camArmPose.orientation.x, camArmPose.orientation.y,
                                  camArmPose.orientation.z, camArmPose.orientation.w])
    r = arrE[0]
    p = arrE[1]
    y = arrE[2]
    p -= 0.2  # turn counterclockwise for about 5deg
    arrQ = quaternion_from_euler(r, p, y)
    pose.orientation.x = arrQ[0]
    pose.orientation.y = arrQ[1]
    pose.orientation.z = arrQ[2]
    pose.orientation.w = arrQ[3]
    if not all_close(pose, camArmPose, .01):
        posePub.publish(pose)
    else:
        rospy.loginfo("Value too small for moving")
    rospy.sleep(1)


def go_to_pose(camPose):
    pose = Pose()
    pose.position.x = camPose.position.x
    pose.position.y = camPose.position.y
    pose.position.z = camPose.position.z
    arrE = euler_from_quaternion([camPose.orientation.x, camPose.orientation.y,
                                  camPose.orientation.z, camPose.orientation.w])
    r = arrE[0]
    p = arrE[1]
    y = arrE[2]
    arrQ = quaternion_from_euler(r, p, y)
    pose.orientation.x = arrQ[0]
    pose.orientation.y = arrQ[1]
    pose.orientation.z = arrQ[2]
    pose.orientation.w = arrQ[3]
    if not all_close(pose, camArmPose, .01):
        posePub.publish(pose)
    else:
        rospy.loginfo("Value too small for moving")
    rospy.sleep(1)


def zoom_view(inOrOut):
    if inOrOut is 1:
        try_view(0.15, 0.125, -0.025, 0.15, 0, 0)
    else:
        try_view(-0.05, -0.125, 0.025, -0.15, 0, 0)
    rospy.sleep(1)


def checkCOM(cCOM, gCOM):
    cx = cCOM[0]
    cy = cCOM[1]
    gx = gCOM[0]
    gy = gCOM[1]
    return (cx in range(0, 292) and cy in range(0, 428) and gx in range(293, 585) and gy in range(429, 857)) or \
           (gx in range(0, 292) and gy in range(0, 428) and cx in range(293, 585) and cy in range(429, 857)) or \
           (cx in range(0, 292) and gy in range(0, 428) and gx in range(293, 585) and cy in range(429, 857)) or \
           (gx in range(0, 292) and cy in range(0, 428) and cx in range(293, 585) and gy in range(429, 857))


def calc_distance(cCOM, gCOM):
    cx = cCOM[0]
    cy = cCOM[1]
    gx = gCOM[0]
    gy = gCOM[1]
    return math.sqrt(math.pow(cx-gx, 2) + math.pow(cy-gy, 2))


# def checkIfZoom(cCOM, gCOM):
#     cx = cCOM[0]
#     cy = cCOM[1]
#     gx = gCOM[0]
#     gy = gCOM[1]
#     return (cx in range(0, 230) and cy in range(0, 320) and gx in range(356, 585) and gy in range(537, 857)) or \
#            (gx in range(0, 230) and gy in range(0, 320) and cx in range(356, 585) and cy in range(537, 857)) or \
#            (cx in range(0, 230) and gy in range(0, 320) and gx in range(356, 585) and cy in range(537, 857)) or \
#            (gx in range(0, 230) and cy in range(0, 320) and cx in range(356, 585) and gy in range(537, 857))


def occlude_state():
    global camArmPose
    rospy.loginfo("occlude state start...")
    pose = camArmPose
    arr = []
    maxArea = -1
    maxNum = -1
    areaO = occluderArea
    areaC = cokeArea
    arr.append(try_view(0.05, 0, 0, 0, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(-0.05, 0, 0, 0, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0.05, 0, 0, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, -0.05, 0, 0, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0.05, 0, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, -0.05, 0, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0, 0.35, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0, -0.35, 0, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0, 0, 0.35, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0, 0, -0.35, 0))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0, 0, 0, 0.35))
    go_to_pose(pose)
    arr.append(try_view(0, 0, 0, 0, 0, -0.35))
    go_to_pose(pose)
    for i in range(len(arr)):
        c, o = arr[i]
        if not (c >= areaC and o <= areaO):
            arr[i] = (-1, -1)
        else:
            maxArea = max(maxArea, c)
            maxNum = i

    if maxArea == -1:
        zoom_view(2)
    else:
        for i in range(len(arr)):
            if i is 0 and i == maxNum:
                try_view(0.05, 0, 0, 0, 0, 0)
            elif i is 1 and i == maxNum:
                try_view(-0.05, 0, 0, 0, 0, 0)
            elif i is 2 and i == maxNum:
                try_view(0, 0.05, 0, 0, 0, 0)
            elif i is 3 and i == maxNum:
                try_view(0, -0.05, 0, 0, 0, 0)
            elif i is 4 and i == maxNum:
                try_view(0, 0, 0.05, 0, 0, 0)
            elif i is 5 and i == maxNum:
                try_view(0, 0, -0.05, 0, 0, 0)
            elif i is 6 and i == maxNum:
                try_view(0, 0, 0, 0.35, 0, 0)
            elif i is 7 and i == maxNum:
                try_view(0, 0, 0, -0.35, 0, 0)
            elif i is 8 and i == maxNum:
                try_view(0, 0, 0, 0, 0.35, 0)
            elif i is 9 and i == maxNum:
                try_view(0, 0, 0, 0, -0.35, 0)
            elif i is 10 and i == maxNum:
                try_view(0, 0, 0, 0, 0, 0.35)
            elif i is 11 and i == maxNum:
                try_view(0, 0, 0, 0, 0, -0.35)

    # rospy.loginfo("____________________________")
    # rospy.loginfo(arr)
    # rospy.loginfo("____________________________")
    # return arr
    rospy.sleep(1)
    rospy.loginfo("occlude state finished...")
    return 1


def approach_state():
    global cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM, BGR_IMAGE
    rospy.loginfo("approach state start...")
    _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)
    distance = calc_distance(cokeCOM, gripperCOM)
    rospy.loginfo(distance)
    while 1:
        if cokeArea >= 0.0 and gripperArea >= 0.0:
            # if checkCOM(cokeCOM, gripperCOM):
            _, _, ccom, _, gcom, _, _ = procImage(BGR_IMAGE)
            if calc_distance(ccom, gcom) < distance - 80:
                zoom_view(1)
                rospy.loginfo("zooming in...")
                rospy.sleep(1)
                _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)
                distance = calc_distance(cokeCOM, gripperCOM)
            elif calc_distance(ccom, gcom) > distance + 120:
                zoom_view(2)
                rospy.loginfo("zooming out...")
                rospy.sleep(1)
                _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)
                distance = calc_distance(cokeCOM, gripperCOM)
            # else:
            #     # adjust_view()
            #     rospy.loginfo("adjust view........")
            if cokeArea >= 10000.0:
                rospy.loginfo("approach state finished...")
                rospy.loginfo("go to conclusion state...")
                return 2
        elif cokeArea >= 5.0 and gripperArea is -1:
            zoom_view(2)
            rospy.loginfo("zooming out...")
            rospy.sleep(1)
            _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)
            distance = calc_distance(cokeCOM, gripperCOM)
        else:
            rospy.loginfo("go to occlusion state...")
            return 3
        rospy.sleep(1)
        _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)


def conclusion_state():
    global cokeArea, gripperArea, occluderArea, occluderCOM, BGR_IMAGE
    rospy.loginfo("conclusion state start...")
    _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)
    while 1:
        if cokeArea >= 10000.0:
            rospy.loginfo("conclusion state finished...")
            return 100
        elif cokeArea >= 0.0 and gripperArea >= 0.0:
            rospy.loginfo("return to approach state...")
            return 1
        else:
            return 3


def run_finite_state_machine():
    global cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM, state, BGR_IMAGE
    setup_state()
    rospy.sleep(5)
    state = 1
    _, cokeArea, cokeCOM, gripperArea, gripperCOM, occluderArea, occluderCOM = procImage(BGR_IMAGE)
    while 1:
        # approach state = 1, conclusion_state = 2, occlude_state = 3, 100 for finished
        if cokeArea >= 0.0 and gripperArea >= 0.0:
            if state is 1:
                state = approach_state()
            elif state is 2:
                # state = conclusion_state()
                rospy.loginfo("conclusion state")
            elif state is 3:
                rospy.loginfo("occlusion state")
                # state = occlude_state()
            elif state is 100:
                rospy.loginfo("Process finished.......")
                break
        else:
            # rospy.loginfo("occlusion state")
            state = occlude_state()


def startNode():
    """
    This function serves to initialize the ROS node.
    """
    rospy.init_node("occlusion_to_pose")
    rospy.Subscriber(camArmPoseTopic, Pose, updatePose)
    rospy.Subscriber(imageTopic, Image, imageCallback)
    rospy.loginfo("occlusion_to_pose node started")
    run_finite_state_machine()
    rospy.spin()


if __name__ == "__main__":
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
