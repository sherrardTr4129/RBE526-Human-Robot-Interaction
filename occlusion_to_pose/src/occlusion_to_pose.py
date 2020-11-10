#!/usr/bin/env python
# Author: Trevor Sherrard
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2

# declare camera arm pose topics
camArmPoseTopic = "/currentCamArmPose"
camArmPoseGoalTopic = "/camArmPoseGoal"

# uncomment appropriate image topic
imageTopic = "/trina2_1/right_arm_cam/color/image_raw"
# imageTopic = "/trina2_1/left_arm_cam/color/image_raw"

# create global Pose message
global camArmPose
camArmPose = Pose()

# set up Pose publisher for newly computed Pose
posePub = rospy.Publisher(camArmPoseGoalTopic, Pose)

def procImage(img):
    """
    This function processes the recieved ROS image
    and calculates an updated pose for the end-effector
    """
    # reference globals so they can be used here
    global camArmPose

    # create new Pose to be used
    goalPose = Pose()

    #TODO: Do image processing here, compute new pose
    
    # return new pose
    return goalPose

def imageCallback(msg):
    """
    This function serves as the ROS subscription callback. It converts
    the raw image data to an OpenCV compatable format.

    params:
        msg -> The recieved ROS image message.
    returns:
        None
    """

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")

    # process image
    goalPose = procImage(orig)

    # publish new pose
    posePub.publish(goalPose)

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

def startNode():
    """
    This function serves to initialize the ROS node.
    """
    rospy.init_node("saliency_to_pose")
    rospy.loginfo("saliency_to_pose node started")

    rospy.Subscriber(camArmPoseTopic, Pose, updatePose)
    rospy.Subscriber(imageTopic, Image, imageCallback)
    rospy.spin()
if __name__ == "__main__":
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
