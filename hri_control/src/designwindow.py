#!/usr/bin/env python

import sys
import random
import numpy as np
from time import sleep
import datetime
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rospy
from sensor_msgs.msg import Joy
from kortex_driver.msg import *
from kortex_driver.srv import *
from design import *

global angleList
global velocityList
global cpList
recordList = []

class ROS(QThread):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        #rospy.Subscriber('/joy',Joy,self.update,queue_size=1,buff_size=52428800)
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.update, queue_size=1, buff_size=52428800)
        self.action_topic_sub = rospy.Subscriber("/my_gen3" + "/action_topic", ActionNotification,
                                                 self.cb_action_topic)
        self.last_action_notif_type = None

        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
        rospy.wait_for_service(play_cartesian_trajectory_full_name)
        self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name,
                                                            PlayCartesianTrajectory)

        play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
        rospy.wait_for_service(play_joint_trajectory_full_name)
        self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)


    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def update(self,base_feedback):
        global angleList
        global velocityList
        global cpList
        Joint0_Angle = base_feedback.actuators[0].position
        Joint1_Angle = base_feedback.actuators[1].position
        Joint2_Angle = base_feedback.actuators[2].position
        Joint3_Angle = base_feedback.actuators[3].position
        Joint4_Angle = base_feedback.actuators[4].position
        Joint5_Angle = base_feedback.actuators[5].position
        Joint6_Angle = base_feedback.actuators[6].position

        Joint0_Vel = base_feedback.actuators[0].velocity
        Joint1_Vel = base_feedback.actuators[1].velocity
        Joint2_Vel = base_feedback.actuators[2].velocity
        Joint3_Vel = base_feedback.actuators[3].velocity
        Joint4_Vel = base_feedback.actuators[4].velocity
        Joint5_Vel = base_feedback.actuators[5].velocity
        Joint6_Vel = base_feedback.actuators[6].velocity

        cpX = base_feedback.base.tool_pose_x
        cpY = base_feedback.base.tool_pose_y
        cpZ = base_feedback.base.tool_pose_z

        angleList = [Joint0_Angle, Joint1_Angle, Joint2_Angle, Joint3_Angle, Joint4_Angle, Joint5_Angle, Joint6_Angle]
        velocityList = [Joint0_Vel, Joint1_Vel, Joint2_Vel, Joint3_Vel, Joint4_Vel, Joint5_Vel, Joint6_Vel]
        cpList = [cpX, cpY, cpZ]

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                updateControlInfo()
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                sleep(0.01)

    def send_joint_angles(self,desireAngle):
        self.last_action_notif_type = None
        # Create the list of angles
        req = PlayJointTrajectoryRequest()
        # Here the arm is vertical (all zeros)
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = desireAngle[i]
            req.input.joint_angles.joint_angles.append(temp_angle)

        # Send the angles
        rospy.loginfo("Sending the robot to angle desired...")
        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def send_cartesian_pose(self, x, y, z):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        # dx,dy,dz should all be something like 0.1-0.5 the unit is probably meter
        req.input.target_pose.x = x
        req.input.target_pose.y = y
        req.input.target_pose.z = z
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.1
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def home_the_robot(self):
        rospy.loginfo("Home robot")
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = 2
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

def updateControlInfo():
    #Slider
    window.horizontalSlider.setValue(angleList[0])
    window.horizontalSlider_2.setValue(angleList[1])
    window.horizontalSlider_3.setValue(angleList[2])
    window.horizontalSlider_4.setValue(angleList[3])
    window.horizontalSlider_5.setValue(angleList[4])
    window.horizontalSlider_6.setValue(angleList[5])
    window.horizontalSlider_7.setValue(angleList[6])


def updateInfo():
    #Angles
    window.label_15.setText("%.2f" % angleList[0])
    window.label_16.setText("%.2f" % angleList[1])
    window.label_17.setText("%.2f" % angleList[2])
    window.label_18.setText("%.2f" % angleList[3])
    window.label_19.setText("%.2f" % angleList[4])
    window.label_20.setText("%.2f" % angleList[5])
    window.label_21.setText("%.2f" % angleList[6])
    #Velocity
    window.label_8.setText("%.2f" % velocityList[0])
    window.label_9.setText("%.2f" % velocityList[1])
    window.label_10.setText("%.2f" % velocityList[2])
    window.label_11.setText("%.2f" % velocityList[3])
    window.label_12.setText("%.2f" % velocityList[4])
    window.label_13.setText("%.2f" % velocityList[5])
    window.label_14.setText("%.2f" % velocityList[6])
    #Cartesian pose
    window.label_35.setText("x_pose: %.2f" % cpList[0])
    window.label_36.setText("y_pose: %.2f" % cpList[1])
    window.label_37.setText("z_pose: %.2f" % cpList[2])
    window.label_38.setText("x_pose: %.2f" % cpList[0])
    window.label_39.setText("y_pose: %.2f" % cpList[1])
    window.label_40.setText("z_pose: %.2f" % cpList[2])
    #Home robot

def home_robot():
    ros.home_the_robot()

def send_cartesian_pose():
    ros.send_cartesian_pose(float(window.lineEdit.text()),float(window.lineEdit_2.text()),float(window.lineEdit_3.text()))
    #ros.example_send_cartesian_pose(0,0,0)

def send_joint_angles():
    da0 = window.horizontalSlider.value() #Desire angle 0
    da1 = window.horizontalSlider_2.value()
    da2 = window.horizontalSlider_3.value()
    da3 = window.horizontalSlider_4.value()
    da4 = window.horizontalSlider_5.value()
    da5 = window.horizontalSlider_6.value()
    da6 = window.horizontalSlider_7.value()
    daList = [da0,da1,da2,da3,da4,da5,da6]
    ros.send_joint_angles(daList)

def recordAngle():

    recordList.append(angleList)
    rospy.loginfo("recording")

def recordCP():
    recordList.append(cpList)
    rospy.loginfo("recording")

def record():
    recordList=[]
    record_timer.start(1000)    # Record angeles every 1 second

def stop():
    record_timer.stop()
    rospy.loginfo("recording terminated")

def play():
    rospy.loginfo("playing")
    for i in recordList:
        #ros.send_joint_angles(i)
        ros.send_cartesian_pose(i[0],i[1],i[2])


if __name__ == "__main__":
    recording = False
    ros = ROS()
    app = QtWidgets.QApplication(sys.argv)
    widget = QtWidgets.QWidget()
    window = Ui_Dialog()
    window.setupUi(widget)
    window.pushButton.clicked.connect(send_cartesian_pose)
    window.pushButton_2.clicked.connect(home_robot)
    window.setAngleButton.clicked.connect(send_joint_angles) #TODO
    window.recordButton.clicked.connect(record)
    window.stopButton.clicked.connect(stop)
    window.playButton.clicked.connect(play)
    window.lineEdit.setText("0")
    window.lineEdit_2.setText("0")
    window.lineEdit_3.setText("0")
    updateControlInfo()
    widget.show()
    timer = QTimer()
    record_timer = QTimer()
    timer.timeout.connect(updateInfo)
    record_timer.timeout.connect(recordCP)
    timer.start(100)

    sys.exit(app.exec_())







